#include "las_fast_reader.hpp"

#include "point_cloud_io_impl.hpp"
#include "point_cloud_traits.hpp"

#include <logging/logging.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace Ddx::point_cloud_io {
namespace {

constexpr std::size_t kChunkBytesTarget = 256ull * 1024ull * 1024ull;

template <typename T>
T read_le(const std::uint8_t* p)
{
	T v{};
	std::memcpy(&v, p, sizeof(T));
	return v;
}

std::uint8_t las_color_to_u8(std::uint16_t value)
{
	return static_cast<std::uint8_t>(value >> 8);
}

bool read_file_prefix(const std::string& file_path, std::vector<std::uint8_t>& buf,
                      std::size_t n)
{
	std::ifstream in(file_path, std::ios::binary);
	if (!in) {
		LOG_ERROR("las_fast: cannot open {}", file_path);
		return false;
	}
	buf.resize(n);
	in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(n));
	const auto got = static_cast<std::size_t>(in.gcount());
	if (got < 227) {
		LOG_ERROR("las_fast: failed reading header from {}", file_path);
		return false;
	}
	buf.resize(got);
	return true;
}

int parse_public_header(const std::vector<std::uint8_t>& buf, LasPublicHeader& header)
{
	if (buf.size() < 227) {
		LOG_ERROR("las_fast: buffer too small for LAS header ({})", buf.size());
		return -1;
	}
	if (std::memcmp(buf.data(), "LASF", 4) != 0) {
		LOG_ERROR("las_fast: missing LASF signature");
		return -1;
	}

	const std::uint8_t ver_major = buf[24];
	const std::uint8_t ver_minor = buf[25];
	header.header_size = read_le<std::uint16_t>(buf.data() + 94);
	header.offset_to_point_data = read_le<std::uint32_t>(buf.data() + 96);
	header.point_data_record_format = buf[104];
	header.point_data_record_length = read_le<std::uint16_t>(buf.data() + 105);
	const std::uint32_t legacy_count = read_le<std::uint32_t>(buf.data() + 107);
	header.scale_x = read_le<double>(buf.data() + 131);
	header.scale_y = read_le<double>(buf.data() + 139);
	header.scale_z = read_le<double>(buf.data() + 147);
	header.offset_x = read_le<double>(buf.data() + 155);
	header.offset_y = read_le<double>(buf.data() + 163);
	header.offset_z = read_le<double>(buf.data() + 171);

	header.point_count = legacy_count;
	if (ver_major == 1 && ver_minor >= 4 && buf.size() >= 255) {
		const std::uint64_t count64 = read_le<std::uint64_t>(buf.data() + 247);
		if (legacy_count == 0 && count64 > 0) {
			header.point_count = count64;
		} else if (count64 > legacy_count) {
			header.point_count = count64;
		}
	}

	if (header.header_size < 227 || header.offset_to_point_data < header.header_size) {
		LOG_ERROR("las_fast: invalid header_size={} offset_to_points={}", header.header_size,
		          header.offset_to_point_data);
		return -1;
	}
	if (header.point_data_record_length < 20) {
		LOG_ERROR("las_fast: point record length too small ({})",
		          header.point_data_record_length);
		return -1;
	}
	if (header.point_count == 0) {
		LOG_ERROR("las_fast: point count is 0");
		return -1;
	}
	if (header.scale_x == 0.0 || header.scale_y == 0.0 || header.scale_z == 0.0) {
		LOG_ERROR("las_fast: scale contains zero");
		return -1;
	}
	return 0;
}

int rgb_field_offset(std::uint8_t point_format)
{
	switch (point_format) {
		case 2:
			return 20;
		case 3:
			return 28;
		case 7:
		case 8:
			return 30;
		default:
			return -1;
	}
}

void world_xyz_from_record(const std::uint8_t* rec, LasPublicHeader const& header, double& x,
                           double& y, double& z)
{
	const std::int32_t xi = read_le<std::int32_t>(rec + 0);
	const std::int32_t yi = read_le<std::int32_t>(rec + 4);
	const std::int32_t zi = read_le<std::int32_t>(rec + 8);
	x = xi * header.scale_x + header.offset_x;
	y = yi * header.scale_y + header.offset_y;
	z = zi * header.scale_z + header.offset_z;
}

bool find_origin_in_records(const std::uint8_t* base, std::uint64_t count, std::uint16_t rec_len,
                            LasPublicHeader const& header, Eigen::Vector3d& origin)
{
	for (std::uint64_t i = 0; i < count; ++i) {
		double x = 0, y = 0, z = 0;
		world_xyz_from_record(base + i * rec_len, header, x, y, z);
		if (is_valid_origin_candidate(x, y, z)) {
			origin = Eigen::Vector3d(x, y, z);
			return true;
		}
	}
	return false;
}

template <typename PointT>
void decode_records_local(const std::uint8_t* base, std::uint64_t count, std::uint64_t index0,
                          std::uint16_t rec_len, LasPublicHeader const& header,
                          Eigen::Vector3d const& origin, int rgb_off, bool file_has_color,
                          pcl::PointCloud<PointT>& cloud)
{
	for (std::uint64_t i = 0; i < count; ++i) {
		const std::uint8_t* rec = base + i * rec_len;
		double wx = 0, wy = 0, wz = 0;
		world_xyz_from_record(rec, header, wx, wy, wz);
		const std::uint16_t intensity = read_le<std::uint16_t>(rec + 12);

		std::uint8_t r = 0;
		std::uint8_t g = 0;
		std::uint8_t b = 0;
		if (file_has_color) {
			r = las_color_to_u8(read_le<std::uint16_t>(rec + rgb_off));
			g = las_color_to_u8(read_le<std::uint16_t>(rec + rgb_off + 2));
			b = las_color_to_u8(read_le<std::uint16_t>(rec + rgb_off + 4));
		}

		PointT& pt = cloud.points[static_cast<std::size_t>(index0 + i)];
		detail::assign_from_las(pt, static_cast<float>(wx - origin.x()),
		                        static_cast<float>(wy - origin.y()),
		                        static_cast<float>(wz - origin.z()),
		                        static_cast<float>(intensity), file_has_color, r, g, b);
	}
}

template <typename PointT>
int load_las_fast_chunked(const std::string& file_path, LasPublicHeader const& header,
                          FramedCloud<PointT>& framed)
{
	const std::uint64_t n = header.point_count;
	const std::uint16_t rec_len = header.point_data_record_length;
	const std::uint64_t bytes_needed =
	    static_cast<std::uint64_t>(header.offset_to_point_data) + n * rec_len;

	std::ifstream in(file_path, std::ios::binary | std::ios::ate);
	if (!in) {
		LOG_ERROR("las_fast: cannot open {}", file_path);
		return -1;
	}
	const auto file_size = static_cast<std::uint64_t>(in.tellg());
	if (file_size < bytes_needed) {
		LOG_ERROR("las_fast: file size {} < expected point block end {}", file_size,
		          bytes_needed);
		return -1;
	}

	const int rgb_off = rgb_field_offset(header.point_data_record_format);
	const bool file_has_color =
	    rgb_off >= 0 && rec_len >= static_cast<std::uint16_t>(rgb_off + 6);

	const std::uint64_t total_point_bytes = n * static_cast<std::uint64_t>(rec_len);
	std::size_t chunk_bytes = kChunkBytesTarget;
	if (chunk_bytes < rec_len) {
		chunk_bytes = rec_len;
	}
	chunk_bytes = (chunk_bytes / rec_len) * rec_len;
	if (chunk_bytes == 0) {
		chunk_bytes = rec_len;
	}
	if (static_cast<std::uint64_t>(chunk_bytes) > total_point_bytes) {
		chunk_bytes = static_cast<std::size_t>(total_point_bytes);
	}

	std::vector<std::uint8_t> chunk(chunk_bytes);
	in.seekg(static_cast<std::streamoff>(header.offset_to_point_data), std::ios::beg);

	Eigen::Vector3d origin = Eigen::Vector3d::Zero();
	bool found_origin = false;
	std::uint64_t points_done = 0;
	while (points_done < n && !found_origin) {
		const std::uint64_t remaining = n - points_done;
		const std::size_t this_records = static_cast<std::size_t>(
		    std::min<std::uint64_t>(remaining, chunk_bytes / rec_len));
		const std::size_t this_bytes = this_records * rec_len;
		in.read(reinterpret_cast<char*>(chunk.data()),
		        static_cast<std::streamsize>(this_bytes));
		if (!in || static_cast<std::size_t>(in.gcount()) != this_bytes) {
			LOG_ERROR("las_fast: failed reading point records from {}", file_path);
			return -1;
		}
		if (find_origin_in_records(chunk.data(), this_records, rec_len, header, origin)) {
			found_origin = true;
		}
		points_done += this_records;
	}
	if (!found_origin) {
		LOG_ERROR("las_fast: no valid origin candidate in {}", file_path);
		return -1;
	}

	framed.origin = origin;
	framed.cloud.clear();
	framed.cloud.points.resize(static_cast<std::size_t>(n));
	framed.cloud.width = static_cast<std::uint32_t>(n);
	framed.cloud.height = 1;
	framed.cloud.is_dense = true;

	in.clear();
	in.seekg(static_cast<std::streamoff>(header.offset_to_point_data), std::ios::beg);
	points_done = 0;
	while (points_done < n) {
		const std::uint64_t remaining = n - points_done;
		const std::size_t this_records = static_cast<std::size_t>(
		    std::min<std::uint64_t>(remaining, chunk_bytes / rec_len));
		const std::size_t this_bytes = this_records * rec_len;
		in.read(reinterpret_cast<char*>(chunk.data()),
		        static_cast<std::streamsize>(this_bytes));
		if (!in || static_cast<std::size_t>(in.gcount()) != this_bytes) {
			LOG_ERROR("las_fast: failed reading point records from {}", file_path);
			framed.cloud.clear();
			return -1;
		}
		decode_records_local(chunk.data(), this_records, points_done, rec_len, header, origin,
		                     rgb_off, file_has_color, framed.cloud);
		points_done += this_records;
	}
	return 0;
}

#ifdef _WIN32
template <typename PointT>
bool try_load_las_fast_mmap(const std::string& file_path, LasPublicHeader const& header,
                            FramedCloud<PointT>& framed)
{
	const std::uint64_t n = header.point_count;
	const std::uint16_t rec_len = header.point_data_record_length;
	const std::uint64_t point_bytes = n * static_cast<std::uint64_t>(rec_len);
	const std::uint64_t map_off = header.offset_to_point_data;

	HANDLE file = CreateFileA(file_path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
	                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
	                          nullptr);
	if (file == INVALID_HANDLE_VALUE) {
		return false;
	}

	LARGE_INTEGER file_size_li{};
	if (!GetFileSizeEx(file, &file_size_li) ||
	    static_cast<std::uint64_t>(file_size_li.QuadPart) < map_off + point_bytes) {
		CloseHandle(file);
		return false;
	}

	HANDLE mapping = CreateFileMappingA(file, nullptr, PAGE_READONLY, 0, 0, nullptr);
	if (!mapping) {
		CloseHandle(file);
		return false;
	}

	SYSTEM_INFO sysinfo{};
	GetSystemInfo(&sysinfo);
	const std::uint64_t gran = sysinfo.dwAllocationGranularity
	                               ? static_cast<std::uint64_t>(sysinfo.dwAllocationGranularity)
	                               : 65536ull;
	const std::uint64_t aligned_off = (map_off / gran) * gran;
	const std::uint64_t view_delta = map_off - aligned_off;
	const std::uint64_t map_span = point_bytes + view_delta;

	DWORD const off_hi = static_cast<DWORD>((aligned_off >> 32) & 0xffffffffu);
	DWORD const off_lo = static_cast<DWORD>(aligned_off & 0xffffffffu);
	void* view = MapViewOfFile(mapping, FILE_MAP_READ, off_hi, off_lo,
	                           static_cast<SIZE_T>(map_span));
	if (!view) {
		CloseHandle(mapping);
		CloseHandle(file);
		return false;
	}

	const int rgb_off = rgb_field_offset(header.point_data_record_format);
	const bool file_has_color =
	    rgb_off >= 0 && rec_len >= static_cast<std::uint16_t>(rgb_off + 6);

	const std::uint8_t* base =
	    static_cast<const std::uint8_t*>(view) + static_cast<std::size_t>(view_delta);

	Eigen::Vector3d origin = Eigen::Vector3d::Zero();
	if (!find_origin_in_records(base, n, rec_len, header, origin)) {
		LOG_ERROR("las_fast: no valid origin candidate in {}", file_path);
		UnmapViewOfFile(view);
		CloseHandle(mapping);
		CloseHandle(file);
		return false;
	}

	framed.origin = origin;
	framed.cloud.clear();
	framed.cloud.points.resize(static_cast<std::size_t>(n));
	framed.cloud.width = static_cast<std::uint32_t>(n);
	framed.cloud.height = 1;
	framed.cloud.is_dense = true;

	decode_records_local(base, n, /*index0=*/0, rec_len, header, origin, rgb_off, file_has_color,
	                     framed.cloud);

	UnmapViewOfFile(view);
	CloseHandle(mapping);
	CloseHandle(file);
	return true;
}
#endif

template <typename PointT>
int load_las_fast_impl(const std::string& file_path, FramedCloud<PointT>& framed)
{
	LasPublicHeader header;
	if (read_las_public_header(file_path, header) != 0) {
		return -1;
	}
	if (!las_fast_eligible(file_path, header)) {
		LOG_ERROR("las_fast: file not eligible (laz or point format {}); path={}",
		          static_cast<unsigned>(header.point_data_record_format), file_path);
		return -1;
	}

#ifdef _WIN32
	if (try_load_las_fast_mmap(file_path, header, framed)) {
		return 0;
	}
	LOG_WARN("las_fast: mmap path failed for {}; falling back to chunked fread", file_path);
#endif
	return load_las_fast_chunked(file_path, header, framed);
}

}  // namespace

int read_las_public_header(const std::string& file_path, LasPublicHeader& header)
{
	std::vector<std::uint8_t> buf;
	if (!read_file_prefix(file_path, buf, 375)) {
		return -1;
	}
	return parse_public_header(buf, header);
}

bool las_fast_eligible(const std::string& file_path, const LasPublicHeader& header)
{
	if (file_extension(file_path) == "laz") {
		return false;
	}
	const std::uint8_t fmt = header.point_data_record_format;
	return fmt <= 3 || (fmt >= 6 && fmt <= 8);
}

int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZ>& framed)
{
	return load_las_fast_impl(file_path, framed);
}

int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZI>& framed)
{
	return load_las_fast_impl(file_path, framed);
}

int load_las_fast(const std::string& file_path, FramedCloud<pcl::PointXYZRGB>& framed)
{
	return load_las_fast_impl(file_path, framed);
}

}  // namespace Ddx::point_cloud_io
