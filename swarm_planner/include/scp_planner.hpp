#pragma once

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

// Others
#include <mission.hpp>
#include <param.hpp>

ILOSTLBEGIN

namespace SwarmPlanning {
    class SCPPlanner {
    public:
        std_msgs::Float64MultiArray msgs_traj_info;
        std_msgs::Float64MultiArray msgs_traj_input;

        SCPPlanner(Mission _mission,
                   Param _param)
                : mission(_mission),
                  param(_param) {
            h = param.time_step;
            T = 34;
            K = round(T / h) + 1;         // number of segments
            N = mission.qn;
            outdim = 3;                 // number of outputs (x,y,z)

            p_max = 5;
            v_max = 10;
            a_max = 10;
            j_max = 10;
            epsilon = 0.01;

            u_prev = Eigen::MatrixXd::Zero(outdim * N * K, 1);
        }

        bool update(bool log) {
            IloEnv env;
            Timer timer;

            u.resize(outdim * N * K);
            try {
                timer.reset();
                buildConstMtx();
                timer.stop();
                ROS_INFO_STREAM("Constraint Matrix runtime: " << timer.elapsedSeconds());

                timer.reset();
                solveQP(env, log);
                timer.stop();
                ROS_INFO_STREAM("QP runtime: " << timer.elapsedSeconds());
            }
            catch (IloException &e) {
                ROS_ERROR_STREAM("Concert exception caught: " << e);
                return false;
            }
            catch (...) {
                ROS_ERROR("Unknown exception caught");
                return false;
            }
            env.end();

            createMsg();
            return true;
        }

    private:
        Mission mission;
        Param param;

        double h, T, p_max, v_max, a_max, j_max, epsilon;
        int K, N, outdim;
        IloNum count_x, count_eq, count_lq;

        Eigen::MatrixXd Q, A_eq, b_eq, A_ineq, b_ineq,
                P, V, A, J, p_start, p_goal,
                A_ineq_dynamics, b_ineq_dynamics, u_prev;
        std::vector<double> u;

        void buildConstMtx() {
            build_Q();
            build_mapping_mtx();
            build_eq_const();
            build_ineq_const();
        }

        void solveQP(const IloEnv &env, bool log) {
            Timer timer;
            IloNum cost_total, cost_prev;

            IloCplex cplex(env);
//        cplex.setParam(IloCplex::Param::TimeLimit, 0.04);

            cost_total = SP_INFINITY;
            cost_prev = 0;
            int iter = 0;

            timer.reset();
            while (abs(cost_total - cost_prev) > epsilon * cost_total) {
                IloModel model(env);
                IloNumVarArray var(env);
                IloRangeArray con(env);

                populatebyrow(model, var, con);

                cplex.extract(model);

                if (log) {
                    cplex.exportModel("/home/jungwon/QPmodel.lp");
                } else {
                    cplex.setOut(env.getNullStream());
                }

                // Optimize the problem and obtain solution.
                if (!cplex.solve()) {
                    env.error() << "Failed to optimize QP, Check ~/.ros/QPresult.txt" << endl;
                    throw (-1);
                }

                IloNumArray vals(env);
                cost_prev = cost_total;
                cost_total = cplex.getObjValue();
                cplex.getValues(vals, var);

                // update input
                for (int dim = 0; dim < outdim; dim++) {
                    for (int qi = 0; qi < N; qi++) {
                        for (int k = 0; k < K; k++) {
                            u[dim * N * K + qi * K + k] = vals[dim * N * K + qi * K + k];
                            u_prev(dim * N * K + qi * K + k, 0) = vals[dim * N * K + qi * K + k];
                        }
                    }
                }

                timer.stop();
                ROS_INFO_STREAM("QP iteration " << iter << " : " << "total_cost : " << cost_total);
                ROS_INFO_STREAM("QP iteration " << iter << " : " << "iteration time : " << timer.elapsedSeconds());
                ROS_INFO_STREAM("QP iteration " << iter << " : " << "x size: " << count_x);
                ROS_INFO_STREAM("QP iteration " << iter << " : " << "eq const size: " << count_eq);
                ROS_INFO_STREAM("QP iteration " << iter << " : " << "ineq const size: " << count_lq);
                iter++;
                timer.reset();

                // update inequality constraints
                update_ineq_const();
            }

            ROS_INFO_STREAM("QP total cost = " << cost_total);
        }

        void createMsg() {
            std::vector<double> traj_info;
            traj_info.emplace_back(N);
            traj_info.emplace_back(K);
            traj_info.emplace_back(h);
            msgs_traj_info.data = traj_info;
            msgs_traj_input.data = u;
        }

        // Cost matrix Q
        void build_Q() {
            Q = Eigen::MatrixXd::Identity(outdim * N * K, outdim * N * K);
        }

        void build_mapping_mtx() {
            P = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // position matrix p = Pu + p_start
            V = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // velocity matrix v = Vu, assume v_start = 0
            A = Eigen::MatrixXd::Identity(outdim * N * K, outdim * N * K); // accelation matrix a = Au
            J = Eigen::MatrixXd::Zero(outdim * N * K, outdim * N * K); // accelation matrix a = Au

            p_start = Eigen::MatrixXd::Zero(outdim * N * K, 1);
            p_goal = Eigen::MatrixXd::Zero(outdim * N, 1);

            for (int dim = 0; dim < outdim; dim++) {
                for (int qi = 0; qi < N; qi++) {
                    int offset = dim * N * K + qi * K;
                    for (int k = 0; k < K; k++) {
                        for (int j = 0; j < k; j++) {
                            P(offset + k, offset + j) = 0.5 * h * h * (2 * (k - j) - 1);
                            V(offset + k, offset + j) = h;
                        }
                        if (k != 0) {
                            J(offset + k, offset + k) = 1 / h;
                            J(offset + k, offset + k - 1) = -1 / h;
                        }

                        p_start(offset + k, 0) = mission.startState[qi][dim];
                    }
                    p_goal(dim * N + qi, 0) = mission.goalState[qi][dim];
                }
            }
        }

        void build_eq_const() {
            A_eq = Eigen::MatrixXd::Zero(4 * outdim * N, outdim * N * K);
            b_eq = Eigen::MatrixXd::Zero(4 * outdim * N, 1);

            Eigen::MatrixXd A_init = Eigen::MatrixXd::Zero(outdim * N, outdim * N * K);
            Eigen::MatrixXd A_final = Eigen::MatrixXd::Zero(outdim * N, outdim * N * K);

            for (int dim = 0; dim < outdim; dim++) {
                for (int qi = 0; qi < N; qi++) {
                    int offset = dim * N * K + qi * K;
                    A_init(dim * N + qi, offset) = 1;
                    A_final(dim * N + qi, offset + K - 1) = 1;
                }
            }

            A_eq.block(0 * outdim * N, 0, outdim * N, outdim * N * K) = A_init;
            A_eq.block(1 * outdim * N, 0, outdim * N, outdim * N * K) = A_final * P;
            A_eq.block(2 * outdim * N, 0, outdim * N, outdim * N * K) = A_final * V;
            A_eq.block(3 * outdim * N, 0, outdim * N, outdim * N * K) = A_final;

            b_eq.block(outdim * N, 0, outdim * N, 1) = p_goal - A_final * p_start;
        }

        void build_ineq_const() {
            A_ineq_dynamics = Eigen::MatrixXd::Zero(8 * outdim * N * K, outdim * N * K);
            A_ineq_dynamics << P,
                    -P,
                    V,
                    -V,
                    A,
                    -A,
                    J,
                    -J;

            b_ineq_dynamics = Eigen::MatrixXd::Zero(8 * outdim * N * K, 1);
            Eigen::MatrixXd b_ineq_base = Eigen::MatrixXd::Ones(outdim * N * K, 1);
            b_ineq_dynamics << b_ineq_base * p_max - p_start,
                    b_ineq_base * p_max + p_start,
                    b_ineq_base * v_max,
                    b_ineq_base * v_max,
                    b_ineq_base * a_max,
                    b_ineq_base * a_max,
                    b_ineq_base * j_max,
                    b_ineq_base * j_max;

            A_ineq = Eigen::MatrixXd::Zero(8 * outdim * N * K, outdim * N * K);
            A_ineq = A_ineq_dynamics;
            b_ineq = Eigen::MatrixXd::Zero(8 * outdim * N * K, 1);
            b_ineq = b_ineq_dynamics;
        }

        void update_ineq_const() {
            Eigen::MatrixXd A_ineq_col = Eigen::MatrixXd::Zero(N * (N - 1) / 2 * K, outdim * N * K);
            Eigen::MatrixXd b_ineq_col = Eigen::MatrixXd::Zero(N * (N - 1) / 2 * K, 1);

            Eigen::MatrixXd picker_i, picker_j, p_i, p_j, p_prev, eta, temp;
            p_prev = P * u_prev + p_start;

            double offset = 0;
            for (int qi = 0; qi < N; qi++) {
                for (int qj = qi + 1; qj < N; qj++) {
                    double R = mission.quad_size[qi] + mission.quad_size[qj];
                    for (int k = 0; k < K; k++) {

                        position_picker(qi, k, picker_i);
                        position_picker(qj, k, picker_j);

                        p_i = picker_i * p_prev;
                        p_j = picker_j * p_prev;

                        double distance = (p_i - p_j).norm();
                        eta = (p_i - p_j) / distance;

                        A_ineq_col.block(offset + k, 0, 1, outdim * N * K) =
                                -eta.transpose() * (picker_i - picker_j) * P;

                        temp = eta.transpose() * ((p_i - p_j) - (picker_i - picker_j) * p_start);
                        b_ineq_col(offset + k, 0) = -(R - distance + temp(0, 0));
                    }
                    offset += K;
                }
            }

            A_ineq = Eigen::MatrixXd::Zero(8 * outdim * N * K + N * (N - 1) / 2 * K, outdim * N * K);
            A_ineq << A_ineq_dynamics,
                    A_ineq_col;
            b_ineq = Eigen::MatrixXd::Zero(8 * outdim * N * K + N * (N - 1) / 2 * K, 1);
            b_ineq << b_ineq_dynamics,
                    b_ineq_col;
        }

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c) {
            IloEnv env = model.getEnv();

            for (int dim = 0; dim < outdim; dim++) {
                for (int qi = 0; qi < N; qi++) {
                    for (int k = 0; k < K; k++) {
                        x.add(IloNumVar(env, -IloInfinity, IloInfinity));

                        std::string name;
                        if (dim == 0) {
                            name = "x_" + std::to_string(qi) + "_" + std::to_string(k);
                        } else if (dim == 1) {
                            name = "y_" + std::to_string(qi) + "_" + std::to_string(k);
                        } else {
                            name = "z_" + std::to_string(qi) + "_" + std::to_string(k);
                        }

                        int row = dim * N * K + qi * K + k;
                        x[row].setName(name.c_str());
                    }
                }
            }
            count_x = x.getSize();

            // Cost function
            IloNumExpr cost(env);
            for (int i = 0; i < Q.rows(); i++) {
                for (int j = 0; j < Q.cols(); j++) {
                    if (Q(i, j) != 0) {
                        cost += Q(i, j) * x[i] * x[j];
                    }
                }
            }
            model.add(IloMinimize(env, cost));


            // Equality Constraints
            for (int i = 0; i < A_eq.rows(); i++) {
                IloNumExpr expr(env);
                for (int j = 0; j < A_eq.cols(); j++) {
                    if (A_eq(i, j) != 0) {
                        expr += A_eq(i, j) * x[j];
                    }
                }
                c.add(expr == b_eq(i));
                expr.end();
            }
            count_eq = c.getSize();

            // Inequality Constraints
            for (int i = 0; i < A_ineq.rows(); i++) {
                IloNumExpr expr(env);
                for (int j = 0; j < A_ineq.cols(); j++) {
                    if (A_ineq(i, j) != 0) {
                        expr += A_ineq(i, j) * x[j];
                    }
                }
                c.add(expr <= b_ineq(i));
                expr.end();
            }
            count_lq = c.getSize() - count_eq;

            model.add(c);
        }

        void position_picker(int qi, int k, Eigen::MatrixXd &P_pick) {
            P_pick = Eigen::MatrixXd::Zero(outdim, outdim * N * K);
            for (int dim = 0; dim < outdim; dim++) {
                P_pick(dim, dim * N * K + qi * K + k) = 1;
            }
        }
    };
}