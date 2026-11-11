// Single-pair TrimICP phase timing: crop / trees / nc_knn / solver.
// Usage:
//   trim_icp_pair_bench <src.pcd> <tgt.pcd> [options]
// Options:
//   --pose tx,ty,tz,qw,qx,qy,qz   initial guess (south string); default Identity
//   --src-n <normals.pcd>        source normals (required for --nc)
//   --tgt-n <normals.pcd>        target normals (required for --nc)
//   --nc                         enable normal-consistency KNN prefilter
//   --no-crop                    disable voxel overlap crop
//   --method ricp|rppl           default ricp
//   --omp N                      omp_set_num_threads(N); 0 = leave default
//   --profile                    enable FRICP inner profile (default off; zero cost when off)
//   --solver-ds                  enable pre-solver voxel first-hit downsample (default on)
//   --no-solver-ds               disable pre-solver downsample
//   --solver-ds-voxel M          downsample voxel size meters (default 0.05)
//   --no-auto-swap               disable |src|>|tgt| internal role swap (default on)
//   --out-smat <path>            write 2-station .smat (MatData): src=I, tgt=T^{-1}
//                                (world=src; p_world = G * p_station)
//
// Prints TrimIcpPhaseTiming after each run.

#include <trim_icp/trim_icp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;

namespace {

bool parsePose7(std::string const& s, Eigen::Matrix4d& T)
{
	std::vector<double> v;
	std::stringstream ss(s);
	std::string tok;
	while (std::getline(ss, tok, ',')) {
		if (tok.empty()) {
			continue;
		}
		v.push_back(std::atof(tok.c_str()));
	}
	if (v.size() < 7) {
		return false;
	}
	Eigen::Quaterniond q(v[3], v[4], v[5], v[6]);  // w,x,y,z
	q.normalize();
	T.setIdentity();
	T.block<3, 3>(0, 0) = q.toRotationMatrix();
	T(0, 3) = v[0];
	T(1, 3) = v[1];
	T(2, 3) = v[2];
	return true;
}

std::string quatWxyz(Eigen::Matrix3d const& R)
{
	Eigen::Quaterniond q(R);
	q.normalize();
	std::ostringstream os;
	os << std::fixed << std::setprecision(9) << q.w() << "," << q.x() << "," << q.y() << ","
	   << q.z();
	return os.str();
}

std::string vec3(Eigen::Vector3d const& t)
{
	std::ostringstream os;
	os << std::fixed << std::setprecision(6) << t.x() << "," << t.y() << "," << t.z();
	return os.str();
}

/// Fine T maps src→tgt (p_tgt = T * p_src). Export world=src: G_src=I, G_tgt=T^{-1}.
bool writePairSmat(std::string const& path, std::string const& src_name,
                   std::string const& tgt_name, Eigen::Matrix4d const& T_src_to_tgt)
{
	Eigen::Matrix4d const G_tgt = T_src_to_tgt.inverse();
	std::ofstream f(path, std::ios::binary | std::ios::out);
	if (!f) {
		return false;
	}
	f << "{\n    \"MatData\": [\n";
	f << "        {\n            \"name\": \"" << src_name << "\",\n"
	  << "            \"rotation\": \"1.000000000,0.000000000,0.000000000,0.000000000\",\n"
	  << "            \"translation\": \"0.000000,0.000000,0.000000\"\n        },\n";
	f << "        {\n            \"name\": \"" << tgt_name << "\",\n"
	  << "            \"rotation\": \"" << quatWxyz(G_tgt.block<3, 3>(0, 0)) << "\",\n"
	  << "            \"translation\": \"" << vec3(G_tgt.block<3, 1>(0, 3)) << "\"\n"
	  << "        }\n";
	f << "    ],\n    \"version\": 1\n}\n";
	return true;
}

void printTiming(char const* label, Ddx::TrimIcpPhaseTiming const& tm)
{
	auto pct = [&](double part) {
		return tm.total_ms > 0.0 ? (100.0 * part / tm.total_ms) : 0.0;
	};
	std::cout << label << " ok=" << tm.ok << " total=" << tm.total_ms << " ms"
	          << " swapped=" << tm.auto_swapped << "\n"
	          << "  crop   " << tm.crop_ms << " ms (" << pct(tm.crop_ms) << "%)"
	          << "  pts " << tm.src_after_crop << "/" << tm.tgt_after_crop
	          << " (from " << tm.src_in << "/" << tm.tgt_in << ")\n"
	          << "  trees  " << tm.trees_ms << " ms (" << pct(tm.trees_ms) << "%)\n"
	          << "  nc_knn " << tm.nc_knn_ms << " ms (" << pct(tm.nc_knn_ms) << "%)"
	          << "  pts " << tm.src_after_nc << "/" << tm.tgt_after_nc << "\n"
	          << "  ds     " << tm.downsample_ms << " ms (" << pct(tm.downsample_ms) << "%)"
	          << "  pts " << tm.src_after_ds << "/" << tm.tgt_after_ds << "\n"
	          << "  solver " << tm.solver_ms << " ms (" << pct(tm.solver_ms) << "%)\n";
	if (tm.profile) {
		std::cout << "  solver_prof stages=" << tm.solver_welsch_stages
		          << " icp_iters=" << tm.solver_icp_iters
		          << " closest_passes=" << tm.solver_closest_passes
		          << " aa_rej=" << tm.solver_aa_rejects << "\n"
		          << "    init=" << tm.solver_init_ms << " ms"
		          << " closest=" << tm.solver_closest_ms << " ms"
		          << " update=" << tm.solver_update_ms << " ms\n";
	}
}

}  // namespace

int main(int argc, char** argv)
{
	if (argc < 3) {
		std::cerr
			<< "Usage: trim_icp_pair_bench <src.pcd> <tgt.pcd> [--pose ...] [--src-n ...] "
			   "[--tgt-n ...] [--nc] [--no-crop] [--method ricp|rppl] [--omp N]\n";
		return 2;
	}

	std::string const src_path = argv[1];
	std::string const tgt_path = argv[2];
	std::string src_n_path;
	std::string tgt_n_path;
	Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
	bool nc = false;
	bool crop = true;
	bool profile = false;
	bool solver_ds = true;
	float solver_ds_voxel = 0.05f;
	bool auto_swap = true;
	std::string out_smat;
	std::string src_name = "001";
	std::string tgt_name = "002";
	auto method = Ddx::registration::BasicICP::Method::RICP;
	int omp_n = 0;

	for (int i = 3; i < argc; ++i) {
		std::string const a = argv[i];
		auto need = [&](char const* opt) -> char const* {
			if (i + 1 >= argc) {
				std::cerr << "missing value for " << opt << "\n";
				std::exit(2);
			}
			return argv[++i];
		};
		if (a == "--pose") {
			if (!parsePose7(need("--pose"), guess)) {
				std::cerr << "bad --pose\n";
				return 2;
			}
		} else if (a == "--src-n") {
			src_n_path = need("--src-n");
		} else if (a == "--tgt-n") {
			tgt_n_path = need("--tgt-n");
		} else if (a == "--nc") {
			nc = true;
		} else if (a == "--no-crop") {
			crop = false;
		} else if (a == "--profile") {
			profile = true;
		} else if (a == "--solver-ds") {
			solver_ds = true;
		} else if (a == "--no-solver-ds") {
			solver_ds = false;
		} else if (a == "--solver-ds-voxel") {
			solver_ds_voxel = static_cast<float>(std::atof(need("--solver-ds-voxel")));
		} else if (a == "--no-auto-swap") {
			auto_swap = false;
		} else if (a == "--out-smat") {
			out_smat = need("--out-smat");
		} else if (a == "--src-name") {
			src_name = need("--src-name");
		} else if (a == "--tgt-name") {
			tgt_name = need("--tgt-name");
		} else if (a == "--method") {
			std::string const m = need("--method");
			if (m == "ricp" || m == "RICP") {
				method = Ddx::registration::BasicICP::Method::RICP;
			} else if (m == "rppl" || m == "RPPL") {
				method = Ddx::registration::BasicICP::Method::RPPL;
			} else {
				std::cerr << "unknown --method " << m << "\n";
				return 2;
			}
		} else if (a == "--omp") {
			omp_n = std::atoi(need("--omp"));
		} else if (a.find(',') != std::string::npos && parsePose7(a, guess)) {
			// backward compat: bare pose string
		} else {
			std::cerr << "unknown arg: " << a << "\n";
			return 2;
		}
	}

#ifdef _OPENMP
	std::cout << "OpenMP max_threads(before)=" << omp_get_max_threads() << "\n";
	if (omp_n > 0) {
		omp_set_num_threads(omp_n);
		std::cout << "OpenMP set_num_threads=" << omp_n
		          << " max_threads(now)=" << omp_get_max_threads() << "\n";
	}
#else
	std::cout << "OpenMP: OFF\n";
	(void)omp_n;
#endif

	Cloud::Ptr src(new Cloud);
	Cloud::Ptr tgt(new Cloud);
	if (pcl::io::loadPCDFile(src_path, *src) < 0 || src->empty()) {
		std::cerr << "load src failed: " << src_path << "\n";
		return 1;
	}
	if (pcl::io::loadPCDFile(tgt_path, *tgt) < 0 || tgt->empty()) {
		std::cerr << "load tgt failed: " << tgt_path << "\n";
		return 1;
	}
	std::cout << "src=" << src->size() << " tgt=" << tgt->size() << " crop=" << crop
	          << " nc=" << nc << " profile=" << profile << " method=" << static_cast<int>(method)
	          << " solver_ds=" << solver_ds << " ds_voxel=" << solver_ds_voxel
	          << " auto_swap=" << auto_swap << "\n";

	NormalCloud::Ptr src_n;
	NormalCloud::Ptr tgt_n;
	if (nc) {
		if (src_n_path.empty() || tgt_n_path.empty()) {
			std::cerr << "--nc requires --src-n and --tgt-n\n";
			return 2;
		}
		src_n.reset(new NormalCloud);
		tgt_n.reset(new NormalCloud);
		if (pcl::io::loadPCDFile(src_n_path, *src_n) < 0 || src_n->size() != src->size()) {
			std::cerr << "bad src normals (need same count as points)\n";
			return 1;
		}
		if (pcl::io::loadPCDFile(tgt_n_path, *tgt_n) < 0 || tgt_n->size() != tgt->size()) {
			std::cerr << "bad tgt normals (need same count as points)\n";
			return 1;
		}
	}

	auto runOnce = [&](char const* label) {
		Ddx::TrimICP icp;
		icp.setSourceCloud(src);
		icp.setTargetCloud(tgt);
		if (src_n) {
			icp.setSourceNormals(src_n);
		}
		if (tgt_n) {
			icp.setTargetNormals(tgt_n);
		}
		icp.setMethod(method);
		icp.setOverlapCropEnabled(crop);
		icp.setOverlapVoxelSize(0.75f);
		icp.setOverlapDilation(1);
		icp.setNormalConsistencyEnabled(nc);
		icp.setProfile(profile);
		icp.setSolverDownsampleEnabled(solver_ds);
		icp.setSolverDownsampleVoxelSize(solver_ds_voxel);
		icp.setAutoRoleSwapEnabled(auto_swap);
		bool const ok = icp.align(guess);
		printTiming(label, icp.lastTiming());
		if (ok) {
			Eigen::Vector3d const t = icp.getFinalTransformation().block<3, 1>(0, 3);
			std::cout << "  t=(" << t.x() << "," << t.y() << "," << t.z() << ")\n";
		}
		return ok;
	};

	// Warm + measure (second call is the one to read for cache-hot phases)
	runOnce("cold");
	bool const warm_ok = runOnce("warm");
	if (warm_ok && !out_smat.empty()) {
		// Re-run once more to get matrix without changing timing prints above...
		// Use last successful align from a fresh run for export.
		Ddx::TrimICP icp;
		icp.setSourceCloud(src);
		icp.setTargetCloud(tgt);
		if (src_n) {
			icp.setSourceNormals(src_n);
		}
		if (tgt_n) {
			icp.setTargetNormals(tgt_n);
		}
		icp.setMethod(method);
		icp.setOverlapCropEnabled(crop);
		icp.setOverlapVoxelSize(0.75f);
		icp.setOverlapDilation(1);
		icp.setNormalConsistencyEnabled(nc);
		icp.setSolverDownsampleEnabled(solver_ds);
		icp.setSolverDownsampleVoxelSize(solver_ds_voxel);
		icp.setAutoRoleSwapEnabled(auto_swap);
		if (!icp.align(guess)) {
			std::cerr << "export align failed\n";
			return 1;
		}
		if (!writePairSmat(out_smat, src_name, tgt_name, icp.getFinalTransformation())) {
			std::cerr << "write smat failed: " << out_smat << "\n";
			return 1;
		}
		std::cout << "wrote smat: " << out_smat << " (world=" << src_name
		          << ", G_tgt=T_src_to_tgt^{-1})\n";
	}
	return 0;
}
