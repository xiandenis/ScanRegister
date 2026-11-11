#include "align_result_loader.hpp"
#include "pose_graph_optimizer.hpp"
#include "se3_math.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

int g_fail = 0;

void expect(bool cond, char const* msg)
{
	if (!cond) {
		std::cerr << "FAIL: " << msg << "\n";
		++g_fail;
	} else {
		std::cout << "OK: " << msg << "\n";
	}
}

Eigen::Matrix4d makeTrans(double x, double y, double z)
{
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T(0, 3) = x;
	T(1, 3) = y;
	T(2, 3) = z;
	return T;
}

/// Convention: G_src = G_tgt * T_rel  ⇒  T_rel = G_tgt^{-1} * G_src
Eigen::Matrix4d relativePose(Eigen::Matrix4d const& T_w_src, Eigen::Matrix4d const& T_w_tgt)
{
	return T_w_tgt.inverse() * T_w_src;
}

double loopResidualNorm(Eigen::Matrix4d const& T_meas,
	Eigen::Matrix4d const& T_w_src,
	Eigen::Matrix4d const& T_w_tgt)
{
	Eigen::Matrix4d const T_pred = T_w_tgt.inverse() * T_w_src;
	return Ddx::se3Log(T_meas.inverse() * T_pred).norm();
}

Eigen::Matrix<double, 6, 6> omegaI(double scale)
{
	return scale * Eigen::Matrix<double, 6, 6>::Identity();
}

} // namespace

int main()
{
	using Ddx::PoseGraphOptimizer;
	using Ddx::se3Log;

	// -------------------------------------------------------------------------
	// 1) Acyclic chain: A(I)–B–C, true relatives, init = truth
	// -------------------------------------------------------------------------
	{
		Eigen::Matrix4d const Ta = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d const Tb = makeTrans(1.0, 0.0, 0.0);
		Eigen::Matrix4d const Tc = makeTrans(2.0, 0.5, 0.0);

		PoseGraphOptimizer opt;
		expect(opt.addNode("A", Ta), "chain: addNode A");
		expect(opt.addNode("B", Tb), "chain: addNode B");
		expect(opt.addNode("C", Tc), "chain: addNode C");

		Eigen::Matrix4d const Tab = relativePose(Tb, Ta); // src=B, tgt=A
		Eigen::Matrix4d const Tbc = relativePose(Tc, Tb); // src=C, tgt=B
		Eigen::Matrix<double, 6, 6> const Omega = omegaI(1.0);
		expect(opt.addEdge("B", "A", Tab, Omega), "chain: edge B→A");
		expect(opt.addEdge("C", "B", Tbc, Omega), "chain: edge C→B");

		expect(opt.optimize(), "chain: optimize succeeds");
		expect(opt.summary().usable, "chain: summary usable");
		expect(opt.summary().final_cost < 1e-12, "chain: final_cost < 1e-12");

		Eigen::Matrix4d Pa, Pb, Pc;
		expect(opt.getPose("A", Pa) && Pa.isApprox(Ta, 1e-9), "chain: pose A ≈ truth");
		expect(opt.getPose("B", Pb) && Pb.isApprox(Tb, 1e-9), "chain: pose B ≈ truth");
		expect(opt.getPose("C", Pc) && Pc.isApprox(Tc, 1e-9), "chain: pose C ≈ truth");

		Eigen::Matrix4d const Tab_hat = relativePose(Pb, Pa);
		Eigen::Matrix4d const Tbc_hat = relativePose(Pc, Pb);
		expect(se3Log(Tab.inverse() * Tab_hat).norm() < 1e-9, "chain: relative B|A err < 1e-9");
		expect(se3Log(Tbc.inverse() * Tbc_hat).norm() < 1e-9, "chain: relative C|B err < 1e-9");

		auto const reports = opt.getEdgeReports();
		expect(reports.size() == 2, "chain: getEdgeReports size 2");
		expect(reports[0].src == "B" && reports[0].tgt == "A", "chain: report0 B→A");
		expect(reports[1].src == "C" && reports[1].tgt == "B", "chain: report1 C→B");
		expect(reports[0].residual.norm() < 1e-9, "chain: report0 residual ≈ 0");
		expect(reports[1].residual.norm() < 1e-9, "chain: report1 residual ≈ 0");
		expect(reports[0].chi2 < 1e-18, "chain: report0 chi2 ≈ 0");
	}

	// -------------------------------------------------------------------------
	// 2) Synthetic loop closure: square 4 stations; noisy closing edge
	// -------------------------------------------------------------------------
	{
		// True square (xy): A(0,0), B(1,0), C(1,1), D(0,1) — used only to build edges
		Eigen::Matrix4d const Ta_true = makeTrans(0.0, 0.0, 0.0);
		Eigen::Matrix4d const Tb_true = makeTrans(1.0, 0.0, 0.0);
		Eigen::Matrix4d const Tc_true = makeTrans(1.0, 1.0, 0.0);
		Eigen::Matrix4d const Td_true = makeTrans(0.0, 1.0, 0.0);

		// Exact chain relatives (3 edges) + noisy loop-closing edge
		Eigen::Matrix4d const Tab = relativePose(Tb_true, Ta_true);
		Eigen::Matrix4d const Tbc = relativePose(Tc_true, Tb_true);
		Eigen::Matrix4d const Tcd = relativePose(Td_true, Tc_true);
		Eigen::Matrix4d Tda_noisy = relativePose(Ta_true, Td_true);
		Tda_noisy(0, 3) += 0.2; // +0.2 m on closing edge

		// Init: propagate along the chain only (G_src = G_tgt * T_rel).
		// Free end D drifts vs the noisy loop measurement; do not seed all stations
		// from full truth maps.
		Eigen::Matrix4d const Ta_init = Ta_true; // lex-min gauge
		Eigen::Matrix4d const Tb_init = Ta_init * Tab;
		Eigen::Matrix4d const Tc_init = Tb_init * Tbc;
		Eigen::Matrix4d const Td_init = Tc_init * Tcd;

		PoseGraphOptimizer opt;
		expect(opt.addNode("A", Ta_init), "loop: addNode A (gauge)");
		expect(opt.addNode("B", Tb_init), "loop: addNode B (chain prop)");
		expect(opt.addNode("C", Tc_init), "loop: addNode C (chain prop)");
		expect(opt.addNode("D", Td_init), "loop: addNode D (chain prop)");

		Eigen::Matrix<double, 6, 6> const Omega = omegaI(1e3);
		expect(opt.addEdge("B", "A", Tab, Omega), "loop: chain edge B→A");
		expect(opt.addEdge("C", "B", Tbc, Omega), "loop: chain edge C→B");
		expect(opt.addEdge("D", "C", Tcd, Omega), "loop: chain edge D→C");
		expect(opt.addEdge("A", "D", Tda_noisy, Omega), "loop: noisy closing A→D");

		double const res_before = loopResidualNorm(Tda_noisy, Ta_init, Td_init);
		expect(res_before > 0.1, "loop: free-end vs noisy loop has nontrivial residual");

		expect(opt.optimize(), "loop: optimize succeeds");
		expect(opt.summary().usable, "loop: summary usable");

		Eigen::Matrix4d Pa, Pb, Pc, Pd;
		expect(opt.getPose("A", Pa), "loop: getPose A");
		expect(opt.getPose("B", Pb), "loop: getPose B");
		expect(opt.getPose("C", Pc), "loop: getPose C");
		expect(opt.getPose("D", Pd), "loop: getPose D");

		// Gauge: lex-min A fixed to chain-propagated initial
		expect((Pa - Ta_init).norm() < 1e-12, "loop: fixed A unchanged (<1e-12)");

		double const res_after = loopResidualNorm(Tda_noisy, Pa, Pd);
		expect(res_after < res_before, "loop: closing residual decreased");

		auto const reports = opt.getEdgeReports();
		expect(reports.size() == 4, "loop: getEdgeReports size 4");
		bool found_close = false;
		for (auto const& r : reports) {
			if (r.src == "A" && r.tgt == "D") {
				found_close = true;
				expect(std::abs(r.residual.norm() - res_after) < 1e-12,
					"loop: closing report residual matches");
				expect(r.translation_error >= 0.0 && r.rotation_error >= 0.0,
					"loop: closing errors non-negative");
			}
		}
		expect(found_close, "loop: closing edge present in getEdgeReports");
	}

	// -------------------------------------------------------------------------
	// 3) Information matrix: strong good edge vs weak bad edge
	// -------------------------------------------------------------------------
	{
		Eigen::Matrix4d const Ta = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d const Tb_true = makeTrans(1.0, 0.0, 0.0);
		Eigen::Matrix4d const T_star = relativePose(Tb_true, Ta);

		Eigen::Matrix4d T_bad = T_star;
		T_bad(0, 3) += 1.0; // extra 1 m

		// Init slightly off so solver has work; still near good edge
		Eigen::Matrix4d const Tb_init = makeTrans(1.2, 0.1, 0.0);

		PoseGraphOptimizer opt;
		expect(opt.addNode("A", Ta), "info: addNode A");
		expect(opt.addNode("B", Tb_init), "info: addNode B");
		expect(opt.addEdge("B", "A", T_star, omegaI(1e6)), "info: strong good edge");
		expect(opt.addEdge("B", "A", T_bad, omegaI(1e-3)), "info: weak bad edge");

		expect(opt.optimize(), "info: optimize succeeds");

		Eigen::Matrix4d Pa, Pb;
		expect(opt.getPose("A", Pa) && opt.getPose("B", Pb), "info: get poses");
		Eigen::Matrix4d const T_hat = relativePose(Pb, Pa);

		double const err_good = se3Log(T_star.inverse() * T_hat).norm();
		double const err_bad = se3Log(T_bad.inverse() * T_hat).norm();
		expect(err_good < 1e-3, "info: relative close to strong edge (<1e-3)");
		expect(err_good < err_bad, "info: closer to good than bad measurement");
	}

	// -------------------------------------------------------------------------
	// 4) Failure paths
	// -------------------------------------------------------------------------
	{
		// Two nodes, no edges → not connected
		{
			PoseGraphOptimizer opt;
			expect(opt.addNode("A", Eigen::Matrix4d::Identity()), "fail: add A");
			expect(opt.addNode("B", makeTrans(1, 0, 0)), "fail: add B");
			expect(!opt.optimize(), "fail: two nodes no edges → false");
			Eigen::Matrix4d Pa;
			expect(opt.getPose("A", Pa) && Pa.isIdentity(1e-15),
				"fail: initials unchanged after disconnected optimize");
		}

		// Three nodes, two edges forming a chain → OK
		{
			PoseGraphOptimizer opt;
			Eigen::Matrix4d const Ta = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d const Tb = makeTrans(1, 0, 0);
			Eigen::Matrix4d const Tc = makeTrans(2, 0, 0);
			expect(opt.addNode("A", Ta) && opt.addNode("B", Tb) && opt.addNode("C", Tc),
				"fail-pos: add A B C");
			expect(opt.addEdge("B", "A", relativePose(Tb, Ta), omegaI(1.0)), "fail-pos: edge BA");
			expect(opt.addEdge("C", "B", relativePose(Tc, Tb), omegaI(1.0)), "fail-pos: edge CB");
			expect(opt.optimize(), "fail-pos: three-node chain OK");
		}

		// Two components: A–B edge + isolated C
		{
			Eigen::Matrix4d const Ta0 = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d const Tb0 = makeTrans(1, 0, 0);
			Eigen::Matrix4d const Tc0 = makeTrans(0, 1, 0);
			PoseGraphOptimizer opt;
			expect(opt.addNode("A", Ta0), "fail2: add A");
			expect(opt.addNode("B", Tb0), "fail2: add B");
			expect(opt.addNode("C", Tc0), "fail2: add C");
			expect(opt.addEdge("B", "A", makeTrans(1, 0, 0), omegaI(1.0)), "fail2: edge BA");
			expect(!opt.optimize(), "fail2: A-B + isolated C → false");
			Eigen::Matrix4d Pa, Pb, Pc;
			expect(opt.getPose("A", Pa) && (Pa - Ta0).norm() < 1e-15, "fail2: A unchanged");
			expect(opt.getPose("B", Pb) && (Pb - Tb0).norm() < 1e-15, "fail2: B unchanged");
			expect(opt.getPose("C", Pc) && (Pc - Tc0).norm() < 1e-15, "fail2: C unchanged");
		}

		// Ω contains NaN → addEdge false
		{
			PoseGraphOptimizer opt;
			expect(opt.addNode("A", Eigen::Matrix4d::Identity()), "fail-nan: add A");
			expect(opt.addNode("B", makeTrans(1, 0, 0)), "fail-nan: add B");
			Eigen::Matrix<double, 6, 6> Omega = omegaI(1.0);
			Omega(2, 2) = std::numeric_limits<double>::quiet_NaN();
			expect(!opt.addEdge("B", "A", makeTrans(1, 0, 0), Omega), "fail-nan: Ω NaN → addEdge false");
		}

		// Duplicate addNode → false
		{
			PoseGraphOptimizer opt;
			expect(opt.addNode("A", Eigen::Matrix4d::Identity()), "fail-dup: first A");
			expect(!opt.addNode("A", makeTrans(1, 0, 0)), "fail-dup: duplicate A → false");
		}
	}

	// -------------------------------------------------------------------------
	// 5) AlignResult information_ + loadFromAlignResults
	// -------------------------------------------------------------------------
	{
		Ddx::AlignResult e;
		e.state_ = Ddx::VALID;
		expect(e.information_.isIdentity(1e-15), "AlignResult: information_ default I");

		// Retained isotropic helper (not used by loader): sim=1,w=1,rms=0.1 → λ=10
		e.similarity_ = 1.0;
		e.weight_ = 1.0;
		e.rms_ = 0.1;
		Eigen::Matrix<double, 6, 6> const Omega_iso = Ddx::isotropicInformationFromAlign(e);
		expect(std::abs(Omega_iso(0, 0) - 10.0) < 1e-12, "isotropic helper: λ=10 (retained unused)");

		e.information_ = 2.5 * Eigen::Matrix<double, 6, 6>::Identity();

		{
			Eigen::Matrix4d const Ta = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d const Tb = makeTrans(1.0, 0.0, 0.0);
			Ddx::AlignResult skip;
			skip.state_ = Ddx::INITIAL;
			skip.src_name_ = "B";
			skip.tgt_name_ = "A";
			skip.matrix_ = relativePose(Tb, Ta);

			Ddx::AlignResult keep = skip;
			keep.state_ = Ddx::VALID;
			keep.information_ = e.information_;

			std::vector<std::string> const names{"A", "B"};
			std::map<std::string, Eigen::Matrix4d> poses{{"A", Ta}, {"B", Tb}};
			std::vector<Ddx::AlignResult> edges{skip, keep};

			PoseGraphOptimizer opt;
			expect(Ddx::loadFromAlignResults(opt, names, poses, edges),
				"loader: loadFromAlignResults OK");
			expect(opt.optimize(), "loader: optimize with AlignResult::information_");
			auto const reports = opt.getEdgeReports();
			expect(reports.size() == 1, "loader: one non-INITIAL edge");
		}
	}

	// Optional: near-π SO(3) Log round-trip (does not block Task 5)
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		double const angle = 3.14159265358979323846 - 1e-4;
		T.block<3, 3>(0, 0) =
			Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
		T(0, 3) = 0.5;
		Eigen::Matrix4d const T2 = Ddx::se3Exp(Ddx::se3Log(T));
		expect((T2 - T).norm() < 1e-6, "optional: near-π SO(3) exp∘log < 1e-6");
	}

	if (g_fail != 0) {
		std::cerr << g_fail << " test(s) failed\n";
		return 1;
	}
	std::cout << "all pose_graph_opt tests passed\n";
	return 0;
}
