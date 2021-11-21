#include <Eigen/Eigen>
#include <fstream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <iostream>
#include <string>

using namespace gtsam;

#define LINESIZE 81920

static Rot3 NormalizedRot3(double w, double x, double y, double z) {
    const double norm = sqrt(w * w + x * x + y * y + z * z), f = 1.0 / norm;
    return Rot3::Quaternion(f * w, f * x, f * y, f * z);
}

GraphAndValues read3DG2oandModifyModel(
    std::string filename, bool userobustmodel, const noiseModel::Robust::shared_ptr& robustmodel) {
    std::ifstream is(filename.c_str());
    if (!is)
        throw invalid_argument("parse3DFactors: can not find file " + filename);
    std::vector<BetweenFactor<Pose3>::shared_ptr> factors;
    while (!is.eof()) {
        char buf[LINESIZE];
        is.getline(buf, LINESIZE);
        istringstream ls(buf);
        string tag;
        ls >> tag;

        if (tag == "EDGE3") {
            Key id1, id2;
            double x, y, z, roll, pitch, yaw;
            ls >> id1 >> id2 >> x >> y >> z >> roll >> pitch >> yaw;
            Matrix m(6, 6);
            for (size_t i = 0; i < 6; i++)
                for (size_t j = i; j < 6; j++)
                    ls >> m(i, j);
            SharedNoiseModel model = noiseModel::Gaussian::Information(m);
            factors.emplace_back(
                new BetweenFactor<Pose3>(id1, id2, Pose3(Rot3::Ypr(yaw, pitch, roll), {x, y, z}), model));
        }
        if (tag == "EDGE_SE3:QUAT") {
            Key id1, id2;
            double x, y, z, qx, qy, qz, qw;
            ls >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            Matrix m(6, 6);
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = i; j < 6; j++) {
                    double mij;
                    ls >> mij;
                    m(i, j) = mij;
                    m(j, i) = mij;
                }
            }
            Matrix mgtsam(6, 6);

            mgtsam.block<3, 3>(0, 0) = m.block<3, 3>(3, 3);  // cov rotation
            mgtsam.block<3, 3>(3, 3) = m.block<3, 3>(0, 0);  // cov translation
            mgtsam.block<3, 3>(0, 3) = m.block<3, 3>(0, 3);  // off diagonal
            mgtsam.block<3, 3>(3, 0) = m.block<3, 3>(3, 0);  // off diagonal
            auto R12 = NormalizedRot3(qw, qx, qy, qz);
            if (userobustmodel && std::abs(static_cast<int>(id2 - id1)) != 1) {
                factors.emplace_back(new BetweenFactor<Pose3>(id1, id2, Pose3(R12, {x, y, z}), robustmodel));
            } else {
                SharedNoiseModel model = noiseModel::Gaussian::Information(mgtsam);
                factors.emplace_back(new BetweenFactor<Pose3>(id1, id2, Pose3(R12, {x, y, z}), model));
            }
        }
    }
    NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);
    for (const auto& factor : factors) {
        graph->push_back(factor);
    }

    const auto poses = parse3DPoses(filename);
    Values::shared_ptr initial(new Values);
    for (const auto& key_pose : poses) {
        initial->insert(key_pose.first, key_pose.second);
    }
    return std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr>(graph, initial);
}

void write3DG2o(const NonlinearFactorGraph& graph, const Values& estimate, const string& filename) {
    fstream stream(filename.c_str(), fstream::out);

    // 3D poses

    for (const auto& key_value : estimate) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p)
            continue;
        const Pose3& pose = p->value();
        const Point3 t = pose.translation();
        const auto q = pose.rotation().toQuaternion();
        stream << "VERTEX_SE3:QUAT " << key_value.key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x()
               << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    // save edges (3D)
    for (const auto& factor_ : graph) {
        boost::shared_ptr<BetweenFactor<Pose3> > factor3D = boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor_);

        if (factor3D) {
            SharedNoiseModel model = factor3D->noiseModel();

            // boost::shared_ptr<noiseModel::Gaussian> gaussianModel =
            //     boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
            // if (!gaussianModel) {
            //     model->print("model\n");
            //     throw invalid_argument("writeG2o: invalid noise model!");
            // }
            // Matrix Info = gaussianModel->R().transpose() * gaussianModel->R();
            const Pose3 pose3D = factor3D->measured();
            const Point3 p = pose3D.translation();
            const auto q = pose3D.rotation().toQuaternion();
            stream << "EDGE_SE3:QUAT " << factor3D->key1() << " " << factor3D->key2() << " " << p.x() << " " << p.y()
                   << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

            Matrix InfoG2o = I_6x6;
            // InfoG2o.block(0, 0, 3, 3) = Info.block(3, 3, 3, 3);  // cov translation
            // InfoG2o.block(3, 3, 3, 3) = Info.block(0, 0, 3, 3);  // cov rotation
            // InfoG2o.block(0, 3, 3, 3) = Info.block(0, 3, 3, 3);  // off diagonal
            // InfoG2o.block(3, 0, 3, 3) = Info.block(3, 0, 3, 3);  // off diagonal

            for (int i = 0; i < 6; i++) {
                for (int j = i; j < 6; j++) {
                    stream << " " << InfoG2o(i, j);
                }
            }
            stream << endl;
        }
    }
    stream.close();
}

int main(int argc, char const* argv[]) {
    int max_iterations = 100;
    std::string g2ofile(argv[1]);
std:;
    string outfile(argv[2]);
    std::cout << "read file " << g2ofile << std::endl;
    bool userobustmodel = true;
    auto prior_model = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    auto noise_model = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    auto loop_model = noiseModel::Diagonal::Variances((Vector(6) << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5).finished());
    auto robust_noise_model = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1), noise_model);
    auto robustLoopModel = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1), loop_model);
    GraphAndValues initial = read3DG2oandModifyModel(g2ofile, userobustmodel, robust_noise_model);
    std::cout << "read file done" << std::endl;
    auto graph_ptr = initial.first;
    auto result_ptr = initial.second;
    // add prior factor to avoid gauge problem
    Pose3 prior_mean;
    graph_ptr->addPrior(0, prior_mean, prior_model);
    // optimizer
    LevenbergMarquardtParams params;
    params.setVerbosity("Termination");  // this will show info about stopping conds
    LevenbergMarquardtOptimizer optimizer(*graph_ptr, *result_ptr, params);
    Values results = optimizer.optimize();
    std::cout << "Optimization complete" << std::endl;
    std::cout << graph_ptr->error(*result_ptr) << std::endl;
    std::cout << graph_ptr->error(results) << std::endl;
    std::cout << "Save the optimized result" << std::endl;
    write3DG2o(*graph_ptr, results, outfile);
    return 0;
}