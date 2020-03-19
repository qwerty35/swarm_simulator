#pragma once

// EIGEN
#include <Eigen/Dense>

// CPLEX
#include <ilcplex/ilocplex.h>

// Submodules
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>
#include <plan_result.hpp>
#include <timer.hpp>

namespace SwarmPlanning {
    class SFCCorridor {
    public:
        SFCCorridor(std::shared_ptr<DynamicEDTOctomap> _distmap_obj,
                    Mission _mission,
                    Param _param)
                : distmap_obj(std::move(_distmap_obj)),
                  mission(std::move(_mission)),
                  param(std::move(_param)) {
            qn = mission.qn;
        }

        bool update(bool _log, SwarmPlanning::PlanResult* _planResult_ptr) {
            planResult_ptr = _planResult_ptr;
            if(planResult_ptr->state != SFC && planResult_ptr->state != OPTIMIZATION){
                return false;
            }
            return updateRelBox();
        }

        bool preprocess(SwarmPlanning::PlanResult* _planResult_ptr){
            planResult_ptr = _planResult_ptr;
            if(planResult_ptr->state != INITTRAJ){
                return false;
            }
            return subdivision();
        }


    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
        Mission mission;
        Param param;
        SwarmPlanning::PlanResult* planResult_ptr;

        int qn, n;
        std::vector<octomap::point3d> samples_x;
        std::vector<double> samples_y;

        bool updateRelBox() {
            Timer timer;
            IloEnv env;
            IloCplex cplex(env);
            cplex.setParam(IloCplex::Param::OptimalityTarget, 2);

            if(planResult_ptr->state == OPTIMIZATION) {
                planResult_ptr->RSFC.clear();
            }
            planResult_ptr->RSFC.resize(qn);
            for (int qi = 0; qi < qn; qi++) {
                planResult_ptr->RSFC[qi].resize(qn);
            }
            for (int qi = 0; qi < qn; qi++) {
                for (int qj = qi + 1; qj < qn; qj++) {
                    if(planResult_ptr->T[qi].size() != planResult_ptr->T[qj].size()){
                        ROS_ERROR("SFCCorridor: All agents must have the same segment times");
                        return false;
                    }
                    int M = planResult_ptr->T[qi].size() - 1;
                    for(int m = 0; m < M; m++) {
                        IloModel model(env);
                        IloNumVarArray var(env);
                        IloRangeArray con(env);

                        populatebyrow(model, var, con, qi, qj, m);
                        cplex.extract(model);
                        if (param.log) {
                            std::string QPmodel_path = param.package_path + "/log/SFC_QPmodel.lp";
                            cplex.exportModel(QPmodel_path.c_str());
                        } else {
                            cplex.setOut(env.getNullStream());
                            cplex.setWarning(env.getNullStream());
                        }

                        // Optimize the problem and obtain solution.
                        if (!cplex.solve()) {
                            ROS_ERROR("SFCCorridor: Failed to optimize QP");
                            throw (-1);
                        }

                        IloNumArray vals(env);
                        cplex.getValues(vals, var);

                        octomap::point3d a(0,0,0);
                        double b = 0;
                        for(int i = 0; i < n; i++) {
                            a += samples_x[i] * (vals[i] * samples_y[i]);
                            b -= samples_y[i];
                            for(int j = 0; j < n; j++){
                                b += vals[j] * samples_y[j] * samples_x[j].dot(samples_x[i]);
                            }
                        }
                        b /= n;

                        RSFC_internal rsfc_ij = {a.normalized(), b/a.norm(), planResult_ptr->T[qi][m], planResult_ptr->T[qi][m+1]};
                        RSFC_internal rsfc_ji = {-a.normalized(), -b/a.norm(), planResult_ptr->T[qi][m], planResult_ptr->T[qi][m+1]};
                        planResult_ptr->RSFC[qi][qj].emplace_back(rsfc_ij);
                        planResult_ptr->RSFC[qj][qi].emplace_back(rsfc_ji);
                    }
                }
            }

            timer.stop();
            ROS_INFO_STREAM("SFCCorridor: SFC for other agents runtime=" << timer.elapsedSeconds());
            planResult_ptr->state = RSFC;
            return true;
        }

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c, int qi, int qj, int m) {
            IloEnv env = model.getEnv();

            samples_x.clear();
            samples_y.clear();
            if(planResult_ptr->state == SFC){
                samples_x.emplace_back(planResult_ptr->initTraj[qi][m]);
                samples_y.emplace_back(-1);
                samples_x.emplace_back(planResult_ptr->initTraj[qi][m+1]);
                samples_y.emplace_back(-1);

                samples_x.emplace_back(planResult_ptr->initTraj[qj][m]);
                samples_y.emplace_back(1);
                samples_x.emplace_back(planResult_ptr->initTraj[qj][m+1]);
                samples_y.emplace_back(1);
            }
            else if(planResult_ptr->state == OPTIMIZATION){
                // sampling for SFC refinement
                int N_samples = 32;
                double t;
                for(int i = 0; i < N_samples; i++){
                    double alpha = (double)i / (double)(N_samples - 1);
                    t = (1 - alpha) * planResult_ptr->T[qi][m] + alpha * planResult_ptr->T[qi][m+1];

                    Eigen::MatrixXd state_i = planResult_ptr->currentState(param, qi, t);
                    octomap::point3d sample_i(state_i(0, 0), state_i(0, 1), state_i(0, 2));
                    samples_x.emplace_back(sample_i);
                    samples_y.emplace_back(-1);

                    Eigen::MatrixXd state_j = planResult_ptr->currentState(param, qj, t);
                    octomap::point3d sample_j(state_j(0, 0), state_j(0, 1), state_j(0, 2));
                    samples_x.emplace_back(sample_j);
                    samples_y.emplace_back(1);
                }
            }

            n = samples_x.size();
            std::string name;
            for(int i = 0; i < n; i++){
                x.add(IloNumVar(env, -IloInfinity, IloInfinity));
                name = "lambda_" + std::to_string(i);
                x[i].setName(name.c_str());
            }

            // Cost function
            IloNumExpr cost(env);
            for (int i = 0; i < n; i++) {
                cost += x[i];
                for(int j = 0; j < n; j++) {
                    cost -= 0.5 * x[i] * x[j] * samples_y[i] * samples_y[j] * samples_x[i].dot(samples_x[j]);
                }
            }
            model.add(IloMaximize(env, cost));

            // Equality Constraints
            IloNumExpr expr(env);
            for (int i = 0; i < n; i++) {
                expr += x[i] * samples_y[i];
            }
            c.add(expr == 0);
            expr.end();

            // Inequality Constraints
            for (int i = 0; i < n; i++) {
                c.add(-x[i] <= 0);
            }

            model.add(c);
        }

        // subdivide initial trajectory to avoid tail-chasing case
        bool subdivision(){
            for(int qi = 0; qi < qn; qi++){
                std::vector<octomap::point3d> initTraj = planResult_ptr->initTraj[qi];
                std::vector<double> T = planResult_ptr->T[qi];

                planResult_ptr->initTraj[qi].clear();
                planResult_ptr->T[qi].clear();

                planResult_ptr->initTraj[qi].emplace_back(initTraj[0]);
                planResult_ptr->T[qi].emplace_back(0);

                for(int m = 1; m < T.size(); m++) {
                    planResult_ptr->initTraj[qi].emplace_back((initTraj[m-1] + initTraj[m]) * 0.5);
                    planResult_ptr->initTraj[qi].emplace_back(initTraj[m]);
                    planResult_ptr->T[qi].emplace_back((T[m-1] + T[m])/2);
                    planResult_ptr->T[qi].emplace_back(T[m]);
                }
            }
        }
    };
}