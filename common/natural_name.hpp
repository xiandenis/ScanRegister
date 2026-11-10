#pragma once

#include <cctype>
#include <string>

namespace Ddx {

/// 站名自然序：数字段按整数值比，其余按字节序。相等仍用 std::string 的完全相等，不用本函数。
inline bool naturalNameLess(std::string const& a, std::string const& b)
{
	std::size_t i = 0;
	std::size_t j = 0;
	while (i < a.size() && j < b.size()) {
		bool const da = std::isdigit(static_cast<unsigned char>(a[i])) != 0;
		bool const db = std::isdigit(static_cast<unsigned char>(b[j])) != 0;
		if (da && db) {
			std::size_t const i0 = i;
			std::size_t const j0 = j;
			while (i < a.size() && std::isdigit(static_cast<unsigned char>(a[i]))) {
				++i;
			}
			while (j < b.size() && std::isdigit(static_cast<unsigned char>(b[j]))) {
				++j;
			}
			std::size_t ia = i0;
			std::size_t jb = j0;
			while (ia < i && a[ia] == '0') {
				++ia;
			}
			while (jb < j && b[jb] == '0') {
				++jb;
			}
			std::size_t const lena = i - ia;
			std::size_t const lenb = j - jb;
			if (lena != lenb) {
				return lena < lenb;
			}
			for (std::size_t k = 0; k < lena; ++k) {
				if (a[ia + k] != b[jb + k]) {
					return a[ia + k] < b[jb + k];
				}
			}
			continue;
		}
		if (a[i] != b[j]) {
			return static_cast<unsigned char>(a[i]) < static_cast<unsigned char>(b[j]);
		}
		++i;
		++j;
	}
	if (i != a.size() || j != b.size()) {
		return i == a.size();
	}
	return a < b;
}

}  // namespace Ddx
