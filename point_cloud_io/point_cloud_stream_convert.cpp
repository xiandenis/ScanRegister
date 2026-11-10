#include "point_cloud_stream_convert.hpp"
#include "las_fast_reader.hpp"

#include <logging/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace Ddx::point_cloud_io {
namespace {

namespace fs = std::filesystem;

constexpr std::uint64_t kMaxChunkBytes = 64ull * 1024ull * 1024ull;
constexpr double kLasScale = 0.001;

struct StreamPoint {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

struct SourceInfo {
  std::uint64_t count = 0;
  bool has_rgb = false;
  LasHeaderOffset las_offset{};
  bool is_las = false;
};

std::size_t clamp_chunk(std::size_t requested)
{
  constexpr std::size_t kPointBytes = 32;
  std::size_t n = requested == 0 ? (1u << 20) : requested;
  const std::size_t cap = static_cast<std::size_t>(kMaxChunkBytes / kPointBytes);
  if (n > cap) {
    n = cap;
  }
  if (n < 1024) {
    n = 1024;
  }
  return n;
}

template <typename T>
T read_le(const std::uint8_t* p)
{
  T v{};
  std::memcpy(&v, p, sizeof(T));
  return v;
}

template <typename T>
void write_le(std::ostream& out, T v)
{
  out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
T read_be(const std::uint8_t* p)
{
  std::uint8_t tmp[sizeof(T)];
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    tmp[i] = p[sizeof(T) - 1 - i];
  }
  return read_le<T>(tmp);
}

bool read_text_line(std::istream& in, std::string& line)
{
  line.clear();
  char c = 0;
  bool any = false;
  while (in.get(c)) {
    any = true;
    if (c == '\n') {
      break;
    }
    if (c != '\r') {
      line.push_back(c);
    }
  }
  return any;
}

std::string lower_copy(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

int scalar_size(const std::string& type)
{
  const std::string t = lower_copy(type);
  if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") {
    return 1;
  }
  if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") {
    return 2;
  }
  if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" ||
      t == "float32") {
    return 4;
  }
  if (t == "double" || t == "float64") {
    return 8;
  }
  return -1;
}

double load_scalar(const std::uint8_t* p, int size, char type, bool big_endian)
{
  const bool is_float = type == 'F' || type == 'f';
  if (size == 4 && is_float) {
    return big_endian ? read_be<float>(p) : read_le<float>(p);
  }
  if (size == 8 && is_float) {
    return big_endian ? read_be<double>(p) : read_le<double>(p);
  }
  if (size == 1) {
    return type == 'I' || type == 'i' ? static_cast<double>(static_cast<std::int8_t>(p[0]))
                                      : static_cast<double>(p[0]);
  }
  if (size == 2) {
    const auto u = big_endian ? read_be<std::uint16_t>(p) : read_le<std::uint16_t>(p);
    if (type == 'I' || type == 'i') {
      return static_cast<double>(static_cast<std::int16_t>(u));
    }
    return static_cast<double>(u);
  }
  if (size == 4) {
    const auto u = big_endian ? read_be<std::uint32_t>(p) : read_le<std::uint32_t>(p);
    if (type == 'I' || type == 'i') {
      return static_cast<double>(static_cast<std::int32_t>(u));
    }
    return static_cast<double>(u);
  }
  if (size == 8) {
    const auto u = big_endian ? read_be<std::uint64_t>(p) : read_le<std::uint64_t>(p);
    return static_cast<double>(u);
  }
  return 0.0;
}

struct Field {
  std::string name;
  int size = 4;
  char type = 'F';
  int count = 1;
  int offset = 0;
  std::uint64_t major_offset = 0;
};

int find_field(const std::vector<Field>& fields, const std::string& name)
{
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (lower_copy(fields[i].name) == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void paint_rgb(StreamPoint& pt, double r, double g, double b, int size)
{
  if (size == 2) {
    pt.r = static_cast<std::uint8_t>(static_cast<unsigned>(r) >> 8);
    pt.g = static_cast<std::uint8_t>(static_cast<unsigned>(g) >> 8);
    pt.b = static_cast<std::uint8_t>(static_cast<unsigned>(b) >> 8);
  } else {
    pt.r = static_cast<std::uint8_t>(r);
    pt.g = static_cast<std::uint8_t>(g);
    pt.b = static_cast<std::uint8_t>(b);
  }
}

void paint_packed_rgb(StreamPoint& pt, float packed)
{
  std::uint32_t u = 0;
  std::memcpy(&u, &packed, sizeof(u));
  pt.r = static_cast<std::uint8_t>((u >> 16) & 0xff);
  pt.g = static_cast<std::uint8_t>((u >> 8) & 0xff);
  pt.b = static_cast<std::uint8_t>(u & 0xff);
}

class ByteSource {
 public:
  explicit ByteSource(std::istream& in, std::uint64_t nbytes) : in_(in), left_(nbytes) {}

  bool get(std::uint8_t& b)
  {
    if (pos_ >= fill_) {
      if (!refill()) {
        return false;
      }
    }
    b = buf_[pos_++];
    return true;
  }

 private:
  bool refill()
  {
    if (left_ == 0) {
      return false;
    }
    const std::size_t n = static_cast<std::size_t>(std::min<std::uint64_t>(left_, buf_.size()));
    in_.read(reinterpret_cast<char*>(buf_.data()), static_cast<std::streamsize>(n));
    const auto got = static_cast<std::size_t>(in_.gcount());
    if (got == 0) {
      return false;
    }
    fill_ = got;
    pos_ = 0;
    left_ -= got;
    return true;
  }

  std::istream& in_;
  std::uint64_t left_ = 0;
  std::vector<std::uint8_t> buf_ = std::vector<std::uint8_t>(64 * 1024);
  std::size_t pos_ = 0;
  std::size_t fill_ = 0;
};

// liblzf stream: back-references stay within 8192 bytes, so only that tail is resident.
bool lzf_decompress_to_file(std::istream& in, std::uint64_t in_len, std::ostream& out,
                            std::uint64_t out_len)
{
  ByteSource src(in, in_len);
  std::vector<std::uint8_t> ring(8192);
  std::uint64_t produced = 0;
  auto emit = [&](std::uint8_t b) -> bool {
    if (produced >= out_len) {
      return false;
    }
    ring[static_cast<std::size_t>(produced % ring.size())] = b;
    out.put(static_cast<char>(b));
    ++produced;
    return static_cast<bool>(out);
  };
  while (produced < out_len) {
    std::uint8_t ctrl = 0;
    if (!src.get(ctrl)) {
      return false;
    }
    if (ctrl < (1u << 5)) {
      const unsigned lit = ctrl + 1u;
      for (unsigned i = 0; i < lit; ++i) {
        std::uint8_t b = 0;
        if (!src.get(b) || !emit(b)) {
          return false;
        }
      }
    } else {
      unsigned len = ctrl >> 5;
      std::uint8_t extra = 0;
      if (len == 7) {
        if (!src.get(extra)) {
          return false;
        }
        len += extra;
      }
      if (!src.get(extra)) {
        return false;
      }
      const unsigned dist = ((ctrl & 0x1fu) << 8) + static_cast<unsigned>(extra) + 1u;
      len += 2;
      if (dist > produced || dist > ring.size()) {
        return false;
      }
      for (unsigned i = 0; i < len; ++i) {
        const std::uint64_t idx = produced - dist;
        const auto b = ring[static_cast<std::size_t>(idx % ring.size())];
        if (!emit(b)) {
          return false;
        }
      }
    }
  }
  return produced == out_len && static_cast<bool>(out);
}

struct TempFile {
  fs::path path;
  explicit TempFile(fs::path p) : path(std::move(p)) {}
  ~TempFile()
  {
    std::error_code ec;
    fs::remove(path, ec);
  }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
};

class ChunkReader {
 public:
  virtual ~ChunkReader() = default;
  SourceInfo info;
  virtual bool next(std::vector<StreamPoint>& out) = 0;
};

class LasChunkReader : public ChunkReader {
 public:
  LasChunkReader(std::ifstream in, LasPublicHeader header, bool subtract, std::size_t chunk)
      : in_(std::move(in)), header_(header), subtract_(subtract), chunk_(chunk)
  {
    info.count = header_.point_count;
    info.is_las = true;
    info.las_offset.x = header_.offset_x;
    info.las_offset.y = header_.offset_y;
    info.las_offset.z = header_.offset_z;
    const int rgb_off = header_.point_data_record_format == 2   ? 20
                        : header_.point_data_record_format == 3 ? 28
                        : (header_.point_data_record_format == 7 ||
                           header_.point_data_record_format == 8)
                            ? 30
                            : -1;
    has_color_ = rgb_off >= 0 && header_.point_data_record_length >= rgb_off + 6;
    rgb_off_ = rgb_off;
    info.has_rgb = has_color_;
    in_.seekg(static_cast<std::streamoff>(header_.offset_to_point_data));
  }

  bool next(std::vector<StreamPoint>& out) override
  {
    out.clear();
    if (index_ >= header_.point_count) {
      return true;
    }
    const std::uint64_t n = std::min<std::uint64_t>(chunk_, header_.point_count - index_);
    const std::uint16_t rec_len = header_.point_data_record_length;
    raw_.resize(static_cast<std::size_t>(n * rec_len));
    in_.read(reinterpret_cast<char*>(raw_.data()), static_cast<std::streamsize>(raw_.size()));
    if (static_cast<std::uint64_t>(in_.gcount()) != raw_.size()) {
      return false;
    }
    const double ox = subtract_ ? header_.offset_x : 0.0;
    const double oy = subtract_ ? header_.offset_y : 0.0;
    const double oz = subtract_ ? header_.offset_z : 0.0;
    out.resize(static_cast<std::size_t>(n));
    for (std::uint64_t i = 0; i < n; ++i) {
      const std::uint8_t* rec = raw_.data() + i * rec_len;
      StreamPoint& pt = out[static_cast<std::size_t>(i)];
      const auto xi = read_le<std::int32_t>(rec);
      const auto yi = read_le<std::int32_t>(rec + 4);
      const auto zi = read_le<std::int32_t>(rec + 8);
      pt.x = static_cast<float>(xi * header_.scale_x + header_.offset_x - ox);
      pt.y = static_cast<float>(yi * header_.scale_y + header_.offset_y - oy);
      pt.z = static_cast<float>(zi * header_.scale_z + header_.offset_z - oz);
      pt.intensity = static_cast<float>(read_le<std::uint16_t>(rec + 12));
      if (has_color_) {
        pt.r = static_cast<std::uint8_t>(read_le<std::uint16_t>(rec + rgb_off_) >> 8);
        pt.g = static_cast<std::uint8_t>(read_le<std::uint16_t>(rec + rgb_off_ + 2) >> 8);
        pt.b = static_cast<std::uint8_t>(read_le<std::uint16_t>(rec + rgb_off_ + 4) >> 8);
      }
    }
    index_ += n;
    return true;
  }

 private:
  std::ifstream in_;
  LasPublicHeader header_;
  bool subtract_ = true;
  std::size_t chunk_ = 0;
  std::uint64_t index_ = 0;
  bool has_color_ = false;
  int rgb_off_ = -1;
  std::vector<std::uint8_t> raw_;
};

bool fill_from_fields(StreamPoint& pt, const std::vector<Field>& fields, const std::uint8_t* rec,
                      bool big_endian)
{
  const int ix = find_field(fields, "x");
  const int iy = find_field(fields, "y");
  const int iz = find_field(fields, "z");
  if (ix < 0 || iy < 0 || iz < 0) {
    return false;
  }
  auto scalar_at = [&](const Field& f) {
    return load_scalar(rec + f.offset, f.size, f.type, big_endian);
  };
  pt.x = static_cast<float>(scalar_at(fields[static_cast<std::size_t>(ix)]));
  pt.y = static_cast<float>(scalar_at(fields[static_cast<std::size_t>(iy)]));
  pt.z = static_cast<float>(scalar_at(fields[static_cast<std::size_t>(iz)]));
  const int ii = find_field(fields, "intensity");
  if (ii >= 0) {
    pt.intensity = static_cast<float>(scalar_at(fields[static_cast<std::size_t>(ii)]));
  }
  const int irgb = find_field(fields, "rgb");
  if (irgb >= 0 && fields[static_cast<std::size_t>(irgb)].size == 4) {
    const float packed = static_cast<float>(scalar_at(fields[static_cast<std::size_t>(irgb)]));
    paint_packed_rgb(pt, packed);
    return true;
  }
  int ir = find_field(fields, "red");
  if (ir < 0) {
    ir = find_field(fields, "r");
  }
  int ig = find_field(fields, "green");
  if (ig < 0) {
    ig = find_field(fields, "g");
  }
  int ib = find_field(fields, "blue");
  if (ib < 0) {
    ib = find_field(fields, "b");
  }
  if (ir >= 0 && ig >= 0 && ib >= 0) {
    paint_rgb(pt, scalar_at(fields[static_cast<std::size_t>(ir)]),
              scalar_at(fields[static_cast<std::size_t>(ig)]),
              scalar_at(fields[static_cast<std::size_t>(ib)]),
              fields[static_cast<std::size_t>(ir)].size);
  }
  return true;
}

class RecordChunkReader : public ChunkReader {
 public:
  RecordChunkReader(std::ifstream in, std::vector<Field> fields, int stride, bool big_endian,
                    std::uint64_t data_pos, std::size_t chunk)
      : in_(std::move(in)),
        fields_(std::move(fields)),
        stride_(stride),
        big_endian_(big_endian),
        chunk_(chunk)
  {
    in_.seekg(static_cast<std::streamoff>(data_pos));
    info.has_rgb = find_field(fields_, "rgb") >= 0 || find_field(fields_, "red") >= 0 ||
                   find_field(fields_, "r") >= 0;
  }

  bool next(std::vector<StreamPoint>& out) override
  {
    out.clear();
    if (index_ >= info.count) {
      return true;
    }
    const std::uint64_t n = std::min<std::uint64_t>(chunk_, info.count - index_);
    raw_.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(stride_));
    in_.read(reinterpret_cast<char*>(raw_.data()), static_cast<std::streamsize>(raw_.size()));
    if (static_cast<std::uint64_t>(in_.gcount()) != raw_.size()) {
      return false;
    }
    out.resize(static_cast<std::size_t>(n));
    for (std::uint64_t i = 0; i < n; ++i) {
      if (!fill_from_fields(out[static_cast<std::size_t>(i)], fields_,
                            raw_.data() + static_cast<std::size_t>(i) * stride_, big_endian_)) {
        return false;
      }
    }
    index_ += n;
    return true;
  }

 private:
  std::ifstream in_;
  std::vector<Field> fields_;
  int stride_ = 0;
  bool big_endian_ = false;
  std::size_t chunk_ = 0;
  std::uint64_t index_ = 0;
  std::vector<std::uint8_t> raw_;
};

class AsciiChunkReader : public ChunkReader {
 public:
  AsciiChunkReader(std::ifstream in, std::vector<Field> fields, std::size_t chunk)
      : in_(std::move(in)), fields_(std::move(fields)), chunk_(chunk)
  {
    info.has_rgb = find_field(fields_, "rgb") >= 0 || find_field(fields_, "red") >= 0 ||
                   find_field(fields_, "r") >= 0;
  }

  bool next(std::vector<StreamPoint>& out) override
  {
    out.clear();
    if (index_ >= info.count) {
      return true;
    }
    const std::uint64_t n = std::min<std::uint64_t>(chunk_, info.count - index_);
    out.reserve(static_cast<std::size_t>(n));
    std::string line;
    for (std::uint64_t i = 0; i < n; ++i) {
      if (!read_text_line(in_, line)) {
        return false;
      }
      std::istringstream ss(line);
      std::vector<double> values;
      values.reserve(fields_.size());
      double v = 0.0;
      while (ss >> v) {
        values.push_back(v);
      }
      if (values.size() < fields_.size()) {
        return false;
      }
      std::vector<std::uint8_t> rec(static_cast<std::size_t>(stride_of(fields_)));
      int off = 0;
      for (std::size_t f = 0; f < fields_.size(); ++f) {
        const Field& field = fields_[f];
        const double value = values[f];
        if (field.size == 4 && (field.type == 'F' || field.type == 'f')) {
          const float fv = static_cast<float>(value);
          std::memcpy(rec.data() + off, &fv, 4);
        } else if (field.size == 8 && (field.type == 'F' || field.type == 'f')) {
          std::memcpy(rec.data() + off, &value, 8);
        } else if (field.size == 1) {
          rec[static_cast<std::size_t>(off)] = static_cast<std::uint8_t>(value);
        } else if (field.size == 2) {
          const auto u = static_cast<std::uint16_t>(value);
          std::memcpy(rec.data() + off, &u, 2);
        } else if (field.size == 4) {
          const auto u = static_cast<std::uint32_t>(value);
          std::memcpy(rec.data() + off, &u, 4);
        }
        off += field.size;
      }
      StreamPoint pt;
      if (!fill_from_fields(pt, fields_, rec.data(), false)) {
        return false;
      }
      out.push_back(pt);
    }
    index_ += n;
    return true;
  }

 private:
  static int stride_of(const std::vector<Field>& fields)
  {
    int n = 0;
    for (const Field& f : fields) {
      n += f.size * f.count;
    }
    return n;
  }

  std::ifstream in_;
  std::vector<Field> fields_;
  std::size_t chunk_ = 0;
  std::uint64_t index_ = 0;
};

class MajorChunkReader : public ChunkReader {
 public:
  MajorChunkReader(std::ifstream in, std::vector<Field> fields, std::unique_ptr<TempFile> tmp,
                   std::size_t chunk)
      : in_(std::move(in)), fields_(std::move(fields)), tmp_(std::move(tmp)), chunk_(chunk)
  {
    info.has_rgb = find_field(fields_, "rgb") >= 0 || find_field(fields_, "red") >= 0 ||
                   find_field(fields_, "r") >= 0;
  }

  bool next(std::vector<StreamPoint>& out) override
  {
    out.clear();
    if (index_ >= info.count) {
      return true;
    }
    const std::uint64_t n = std::min<std::uint64_t>(chunk_, info.count - index_);
    out.resize(static_cast<std::size_t>(n));
    std::vector<std::uint8_t> col;
    for (const Field& field : fields_) {
      col.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(field.size));
      const std::uint64_t pos = field.major_offset + index_ * static_cast<std::uint64_t>(field.size);
      in_.seekg(static_cast<std::streamoff>(pos));
      in_.read(reinterpret_cast<char*>(col.data()), static_cast<std::streamsize>(col.size()));
      if (static_cast<std::uint64_t>(in_.gcount()) != col.size()) {
        return false;
      }
      columns_.push_back(std::move(col));
      col.clear();
    }
    if (columns_.size() != fields_.size()) {
      return false;
    }
    for (std::uint64_t i = 0; i < n; ++i) {
      scratch_.assign(static_cast<std::size_t>(stride_), 0);
      for (std::size_t f = 0; f < fields_.size(); ++f) {
        const Field& field = fields_[f];
        std::memcpy(scratch_.data() + field.offset,
                    columns_[f].data() + static_cast<std::size_t>(i) * field.size,
                    static_cast<std::size_t>(field.size));
      }
      if (!fill_from_fields(out[static_cast<std::size_t>(i)], fields_, scratch_.data(), false)) {
        columns_.clear();
        return false;
      }
    }
    columns_.clear();
    index_ += n;
    return true;
  }

  void set_stride(int stride) { stride_ = stride; }

 private:
  std::ifstream in_;
  std::vector<Field> fields_;
  std::unique_ptr<TempFile> tmp_;
  std::size_t chunk_ = 0;
  std::uint64_t index_ = 0;
  int stride_ = 0;
  std::vector<std::uint8_t> scratch_;
  std::vector<std::vector<std::uint8_t>> columns_;
};

std::ifstream open_in(const std::string& path)
{
  return std::ifstream(fs::path(path), std::ios::binary);
}

std::unique_ptr<ChunkReader> open_las(const std::string& path, bool subtract, std::size_t chunk)
{
  if (file_extension(path) == "laz") {
    LOG_ERROR("stream convert refuses LAZ (would load the whole cloud): {}", path);
    return nullptr;
  }
  LasPublicHeader header;
  if (read_las_public_header(path, header) != 0) {
    return nullptr;
  }
  if (!las_fast_eligible(path, header)) {
    LOG_ERROR("stream convert supports uncompressed LAS formats 0-3 only: {}", path);
    return nullptr;
  }
  if (header.point_count == 0) {
    LOG_ERROR("stream convert refusing empty LAS: {}", path);
    return nullptr;
  }
  std::ifstream in = open_in(path);
  if (!in) {
    LOG_ERROR("stream convert cannot open {}", path);
    return nullptr;
  }
  return std::make_unique<LasChunkReader>(std::move(in), header, subtract, chunk);
}

bool parse_pcd_header(std::istream& in, std::vector<Field>& fields, std::uint64_t& count,
                      std::string& data_kind)
{
  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::string line;
  data_kind.clear();
  while (read_text_line(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    key = lower_copy(key);
    if (key == "fields") {
      std::string name;
      while (ss >> name) {
        names.push_back(name);
      }
    } else if (key == "size") {
      int v = 0;
      while (ss >> v) {
        sizes.push_back(v);
      }
    } else if (key == "type") {
      std::string v;
      while (ss >> v) {
        types.push_back(v.empty() ? 'F' : v[0]);
      }
    } else if (key == "count") {
      int v = 0;
      while (ss >> v) {
        counts.push_back(v);
      }
    } else if (key == "points") {
      ss >> count;
    } else if (key == "data") {
      ss >> data_kind;
      data_kind = lower_copy(data_kind);
      break;
    }
  }
  if (names.empty() || names.size() != sizes.size() || names.size() != types.size()) {
    return false;
  }
  if (counts.empty()) {
    counts.assign(names.size(), 1);
  }
  if (counts.size() != names.size()) {
    return false;
  }
  fields.clear();
  int off = 0;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (counts[i] != 1) {
      return false;
    }
    Field f;
    f.name = names[i];
    f.size = sizes[i];
    f.type = types[i];
    f.count = 1;
    f.offset = off;
    off += f.size;
    fields.push_back(f);
  }
  return find_field(fields, "x") >= 0 && count > 0 && !data_kind.empty();
}

std::unique_ptr<ChunkReader> open_pcd(const std::string& path, std::size_t chunk)
{
  std::ifstream in = open_in(path);
  if (!in) {
    LOG_ERROR("stream convert cannot open {}", path);
    return nullptr;
  }
  std::vector<Field> fields;
  std::uint64_t count = 0;
  std::string kind;
  if (!parse_pcd_header(in, fields, count, kind)) {
    LOG_ERROR("stream convert failed to parse PCD header: {}", path);
    return nullptr;
  }
  const auto data_pos = static_cast<std::uint64_t>(in.tellg());
  if (kind == "binary") {
    int stride = 0;
    for (const Field& f : fields) {
      stride += f.size;
    }
    auto reader = std::make_unique<RecordChunkReader>(std::move(in), fields, stride, false, data_pos,
                                                      chunk);
    reader->info.count = count;
    return reader;
  }
  if (kind == "ascii") {
    auto reader = std::make_unique<AsciiChunkReader>(std::move(in), fields, chunk);
    reader->info.count = count;
    return reader;
  }
  if (kind != "binary_compressed") {
    LOG_ERROR("stream convert unsupported PCD DATA {}: {}", kind, path);
    return nullptr;
  }
  std::uint32_t comp_size = 0;
  std::uint32_t raw_size = 0;
  in.read(reinterpret_cast<char*>(&comp_size), 4);
  in.read(reinterpret_cast<char*>(&raw_size), 4);
  if (!in) {
    LOG_ERROR("stream convert truncated compressed PCD: {}", path);
    return nullptr;
  }
  auto tmp = std::make_unique<TempFile>(fs::temp_directory_path() /
                                        ("rtr_pcd_lzf_" + std::to_string(raw_size) + ".bin"));
  {
    std::ofstream raw(tmp->path, std::ios::binary);
    if (!raw || !lzf_decompress_to_file(in, comp_size, raw, raw_size)) {
      LOG_ERROR("stream convert LZF decompress failed: {}", path);
      return nullptr;
    }
  }
  int stride = 0;
  std::uint64_t major = 0;
  for (Field& f : fields) {
    f.major_offset = major;
    major += static_cast<std::uint64_t>(f.size) * count;
    stride += f.size;
  }
  if (major != raw_size) {
    LOG_ERROR("stream convert compressed PCD size mismatch ({} vs {}): {}", major, raw_size, path);
    return nullptr;
  }
  std::ifstream raw_in(tmp->path, std::ios::binary);
  if (!raw_in) {
    return nullptr;
  }
  auto reader = std::make_unique<MajorChunkReader>(std::move(raw_in), fields, std::move(tmp), chunk);
  reader->set_stride(stride);
  reader->info.count = count;
  return reader;
}

struct PlyProp {
  std::string name;
  std::string type;
  int size = 0;
  int offset = 0;
};

struct PlyElem {
  std::string name;
  std::uint64_t count = 0;
  std::vector<PlyProp> props;
  bool has_list = false;
  int stride = 0;
};

std::unique_ptr<ChunkReader> open_ply(const std::string& path, std::size_t chunk)
{
  std::ifstream in = open_in(path);
  if (!in) {
    LOG_ERROR("stream convert cannot open {}", path);
    return nullptr;
  }
  std::string line;
  if (!read_text_line(in, line) || lower_copy(line) != "ply") {
    LOG_ERROR("stream convert not a PLY: {}", path);
    return nullptr;
  }
  bool binary = false;
  bool big_endian = false;
  bool ascii = false;
  std::vector<PlyElem> elems;
  while (read_text_line(in, line)) {
    if (line.empty() || line.rfind("comment", 0) == 0 || line.rfind("obj_info", 0) == 0) {
      continue;
    }
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    key = lower_copy(key);
    if (key == "format") {
      std::string fmt;
      ss >> fmt;
      fmt = lower_copy(fmt);
      if (fmt == "ascii") {
        ascii = true;
      } else if (fmt == "binary_little_endian") {
        binary = true;
      } else if (fmt == "binary_big_endian") {
        binary = true;
        big_endian = true;
      } else {
        LOG_ERROR("stream convert unsupported PLY format {}: {}", fmt, path);
        return nullptr;
      }
    } else if (key == "element") {
      PlyElem e;
      ss >> e.name >> e.count;
      elems.push_back(std::move(e));
    } else if (key == "property") {
      if (elems.empty()) {
        return nullptr;
      }
      std::string t0;
      ss >> t0;
      if (lower_copy(t0) == "list") {
        elems.back().has_list = true;
        continue;
      }
      PlyProp p;
      p.type = t0;
      ss >> p.name;
      p.size = scalar_size(p.type);
      if (p.size < 0) {
        LOG_ERROR("stream convert unsupported PLY property type {}: {}", p.type, path);
        return nullptr;
      }
      p.offset = elems.back().stride;
      elems.back().stride += p.size;
      elems.back().props.push_back(std::move(p));
    } else if (key == "end_header") {
      break;
    }
  }
  const auto vertex = std::find_if(elems.begin(), elems.end(), [](const PlyElem& e) {
    return lower_copy(e.name) == "vertex";
  });
  if (vertex == elems.end() || vertex->count == 0 || vertex->has_list) {
    LOG_ERROR("stream convert PLY has no vertex cloud: {}", path);
    return nullptr;
  }
  for (auto it = elems.begin(); it != vertex; ++it) {
    if (it->has_list) {
      LOG_ERROR("stream convert cannot skip listed PLY element before vertex: {}", path);
      return nullptr;
    }
    if (ascii) {
      std::string skip;
      for (std::uint64_t i = 0; i < it->count; ++i) {
        if (!read_text_line(in, skip)) {
          return nullptr;
        }
      }
    } else {
      const std::uint64_t bytes = static_cast<std::uint64_t>(it->stride) * it->count;
      in.seekg(static_cast<std::streamoff>(bytes), std::ios::cur);
      if (!in) {
        return nullptr;
      }
    }
  }
  std::vector<Field> fields;
  char type_code = 'F';
  for (const PlyProp& p : vertex->props) {
    Field f;
    f.name = p.name;
    f.size = p.size;
    f.offset = p.offset;
    const std::string t = lower_copy(p.type);
    if (t == "float" || t == "float32" || t == "double" || t == "float64") {
      f.type = 'F';
    } else if (t == "char" || t == "short" || t == "int" || t == "int8" || t == "int16" ||
               t == "int32") {
      f.type = 'I';
    } else {
      f.type = 'U';
    }
    (void)type_code;
    fields.push_back(f);
  }
  if (ascii) {
    auto reader = std::make_unique<AsciiChunkReader>(std::move(in), fields, chunk);
    reader->info.count = vertex->count;
    return reader;
  }
  const auto data_pos = static_cast<std::uint64_t>(in.tellg());
  auto reader = std::make_unique<RecordChunkReader>(std::move(in), fields, vertex->stride, big_endian,
                                                    data_pos, chunk);
  reader->info.count = vertex->count;
  return reader;
}

class ChunkWriter {
 public:
  virtual ~ChunkWriter() = default;
  virtual bool write(const std::vector<StreamPoint>& pts) = 0;
  virtual bool finish() = 0;
};

class PcdWriter : public ChunkWriter {
 public:
  PcdWriter(std::ofstream out, std::uint64_t count, bool rgb) : out_(std::move(out))
  {
    out_ << "# .PCD v0.7\nVERSION 0.7\n";
    if (rgb) {
      out_ << "FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\n";
    } else {
      out_ << "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n";
    }
    out_ << "WIDTH " << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count
         << "\nDATA binary\n";
    rgb_ = rgb;
  }

  bool write(const std::vector<StreamPoint>& pts) override
  {
    for (const StreamPoint& pt : pts) {
      write_le<float>(out_, pt.x);
      write_le<float>(out_, pt.y);
      write_le<float>(out_, pt.z);
      write_le<float>(out_, pt.intensity);
      if (rgb_) {
        const std::uint32_t packed = (static_cast<std::uint32_t>(pt.r) << 16) |
                                    (static_cast<std::uint32_t>(pt.g) << 8) |
                                    static_cast<std::uint32_t>(pt.b);
        float bits = 0.f;
        std::memcpy(&bits, &packed, sizeof(bits));
        write_le<float>(out_, bits);
      }
    }
    return static_cast<bool>(out_);
  }

  bool finish() override { return static_cast<bool>(out_); }

 private:
  std::ofstream out_;
  bool rgb_ = false;
};

class PlyWriter : public ChunkWriter {
 public:
  PlyWriter(std::ofstream out, std::uint64_t count, bool rgb) : out_(std::move(out))
  {
    out_ << "ply\nformat binary_little_endian 1.0\n";
    out_ << "element vertex " << count << "\n";
    out_ << "property float x\nproperty float y\nproperty float z\nproperty float intensity\n";
    if (rgb) {
      out_ << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    out_ << "end_header\n";
    rgb_ = rgb;
  }

  bool write(const std::vector<StreamPoint>& pts) override
  {
    for (const StreamPoint& pt : pts) {
      write_le<float>(out_, pt.x);
      write_le<float>(out_, pt.y);
      write_le<float>(out_, pt.z);
      write_le<float>(out_, pt.intensity);
      if (rgb_) {
        out_.put(static_cast<char>(pt.r));
        out_.put(static_cast<char>(pt.g));
        out_.put(static_cast<char>(pt.b));
      }
    }
    return static_cast<bool>(out_);
  }

  bool finish() override { return static_cast<bool>(out_); }

 private:
  std::ofstream out_;
  bool rgb_ = false;
};

class LasWriter : public ChunkWriter {
 public:
  LasWriter(std::ofstream out, std::uint64_t count, bool rgb, LasHeaderOffset offset)
      : out_(std::move(out)), count_(count), rgb_(rgb), offset_(offset)
  {
    rec_len_ = rgb_ ? 26 : 20;
    std::vector<char> header(227, 0);
    std::memcpy(header.data(), "LASF", 4);
    header[24] = 1;
    header[25] = 2;
    const auto hs = static_cast<std::uint16_t>(227);
    std::memcpy(header.data() + 94, &hs, 2);
    const auto off = static_cast<std::uint32_t>(227);
    std::memcpy(header.data() + 96, &off, 4);
    header[104] = static_cast<char>(rgb_ ? 2 : 0);
    std::memcpy(header.data() + 105, &rec_len_, 2);
    const auto n32 = static_cast<std::uint32_t>(count_);
    std::memcpy(header.data() + 107, &n32, 4);
    std::memcpy(header.data() + 111, &n32, 4);
    std::memcpy(header.data() + 131, &kLasScale, 8);
    std::memcpy(header.data() + 139, &kLasScale, 8);
    std::memcpy(header.data() + 147, &kLasScale, 8);
    std::memcpy(header.data() + 155, &offset_.x, 8);
    std::memcpy(header.data() + 163, &offset_.y, 8);
    std::memcpy(header.data() + 171, &offset_.z, 8);
    out_.write(header.data(), static_cast<std::streamsize>(header.size()));
    min_x_ = min_y_ = min_z_ = std::numeric_limits<double>::infinity();
    max_x_ = max_y_ = max_z_ = -std::numeric_limits<double>::infinity();
  }

  bool write(const std::vector<StreamPoint>& pts) override
  {
    for (const StreamPoint& pt : pts) {
      const double x = pt.x;
      const double y = pt.y;
      const double z = pt.z;
      min_x_ = std::min(min_x_, x);
      min_y_ = std::min(min_y_, y);
      min_z_ = std::min(min_z_, z);
      max_x_ = std::max(max_x_, x);
      max_y_ = std::max(max_y_, y);
      max_z_ = std::max(max_z_, z);
      const auto qx = static_cast<std::int32_t>(std::llround(x / kLasScale));
      const auto qy = static_cast<std::int32_t>(std::llround(y / kLasScale));
      const auto qz = static_cast<std::int32_t>(std::llround(z / kLasScale));
      write_le<std::int32_t>(out_, qx);
      write_le<std::int32_t>(out_, qy);
      write_le<std::int32_t>(out_, qz);
      const double inten_d = pt.intensity < 0.f ? 0.0 : (pt.intensity > 65535.f ? 65535.0 : pt.intensity);
      const auto inten = static_cast<std::uint16_t>(inten_d);
      write_le<std::uint16_t>(out_, inten);
      out_.put(0);
      out_.put(0);
      out_.put(0);
      out_.put(0);
      write_le<std::uint16_t>(out_, 0);
      if (rgb_) {
        write_le<std::uint16_t>(out_, static_cast<std::uint16_t>(pt.r) << 8);
        write_le<std::uint16_t>(out_, static_cast<std::uint16_t>(pt.g) << 8);
        write_le<std::uint16_t>(out_, static_cast<std::uint16_t>(pt.b) << 8);
      }
    }
    return static_cast<bool>(out_);
  }

  bool finish() override
  {
    auto patch = [&](int at, double v) {
      out_.seekp(at);
      write_le<double>(out_, v);
    };
    patch(179, max_x_);
    patch(187, min_x_);
    patch(195, max_y_);
    patch(203, min_y_);
    patch(211, max_z_);
    patch(219, min_z_);
    return static_cast<bool>(out_);
  }

 private:
  std::ofstream out_;
  std::uint64_t count_ = 0;
  bool rgb_ = false;
  LasHeaderOffset offset_{};
  std::uint16_t rec_len_ = 20;
  double min_x_ = 0, min_y_ = 0, min_z_ = 0, max_x_ = 0, max_y_ = 0, max_z_ = 0;
};

std::unique_ptr<ChunkWriter> open_writer(const std::string& path, PointCloudFormat fmt,
                                         const SourceInfo& info, const StreamConvertOptions& opt)
{
  if (info.count == 0) {
    LOG_ERROR("stream convert refusing empty source");
    return nullptr;
  }
  std::ofstream out(fs::path(path), std::ios::binary);
  if (!out) {
    LOG_ERROR("stream convert cannot create {}", path);
    return nullptr;
  }
  if (fmt == PointCloudFormat::Pcd) {
    return std::make_unique<PcdWriter>(std::move(out), info.count, info.has_rgb);
  }
  if (fmt == PointCloudFormat::Ply) {
    return std::make_unique<PlyWriter>(std::move(out), info.count, info.has_rgb);
  }
  if (fmt == PointCloudFormat::Las || fmt == PointCloudFormat::Slas) {
    if (info.count > std::numeric_limits<std::uint32_t>::max()) {
      LOG_ERROR("stream convert LAS output point count exceeds uint32: {}", path);
      return nullptr;
    }
    LasHeaderOffset offset{};
    if (opt.subtract_las_header_offset && opt.preserve_source_las_offset && info.is_las) {
      offset = info.las_offset;
    }
    return std::make_unique<LasWriter>(std::move(out), info.count, info.has_rgb, offset);
  }
  LOG_ERROR("stream convert unsupported destination: {}", path);
  return nullptr;
}

}  // namespace

int convert_point_cloud(const std::string& src_path, const std::string& dst_path,
                        const StreamConvertOptions& options)
{
  if (src_path == dst_path) {
    LOG_ERROR("stream convert source and destination are the same: {}", src_path);
    return -1;
  }
  const PointCloudFormat src_fmt = detect_format(src_path);
  const PointCloudFormat dst_fmt = detect_format(dst_path);
  if (src_fmt == PointCloudFormat::Unknown || dst_fmt == PointCloudFormat::Unknown) {
    LOG_ERROR("stream convert needs .pcd/.ply/.las/.slas: {} -> {}", src_path, dst_path);
    return -1;
  }
  const std::size_t chunk = clamp_chunk(options.chunk_points);
  std::unique_ptr<ChunkReader> reader;
  if (src_fmt == PointCloudFormat::Las || src_fmt == PointCloudFormat::Slas) {
    reader = open_las(src_path, options.subtract_las_header_offset, chunk);
  } else if (src_fmt == PointCloudFormat::Pcd) {
    reader = open_pcd(src_path, chunk);
  } else if (src_fmt == PointCloudFormat::Ply) {
    reader = open_ply(src_path, chunk);
  }
  if (!reader) {
    return -1;
  }
  auto writer = open_writer(dst_path, dst_fmt, reader->info, options);
  if (!writer) {
    return -1;
  }
  std::vector<StreamPoint> batch;
  std::uint64_t seen = 0;
  while (seen < reader->info.count) {
    if (!reader->next(batch)) {
      LOG_ERROR("stream convert read failed: {}", src_path);
      return -1;
    }
    if (batch.empty()) {
      LOG_ERROR("stream convert short read: {}", src_path);
      return -1;
    }
    if (!writer->write(batch)) {
      LOG_ERROR("stream convert write failed: {}", dst_path);
      return -1;
    }
    seen += batch.size();
  }
  if (!writer->finish()) {
    LOG_ERROR("stream convert finalize failed: {}", dst_path);
    return -1;
  }
  return 0;
}

}  // namespace Ddx::point_cloud_io
