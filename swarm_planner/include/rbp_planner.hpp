#pragma once

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

// Submodules
#include <rbp_corridor.hpp>
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

ILOSTLBEGIN

namespace SwarmPlanning {
    class RBPPlanner {
    public:
        RBPPlanner(Mission _mission,
                   Param _param)
                : mission(std::move(_mission)),
                  param(std::move(_param)) {
            n = param.n; // degree of polynomial
            phi = param.phi; // desired derivatives
            N = mission.qn; // the number of agents
            outdim = 3; // the number of outputs (x,y,z)

            setBatch(0);
        }

        bool update(bool log, SwarmPlanning::PlanResult* _planResult_ptr) {
            planResult_ptr = _planResult_ptr;
            M = planResult_ptr->T.size() - 1; // the number of segments
            offset_quad = M * (n + 1);
            offset_seg = n + 1;

            coef.resize(N);
            for (int qi = 0; qi < N; qi++) {
                coef[qi] = Eigen::MatrixXd::Zero(offset_quad, outdim);
            }

            IloEnv env;
            Timer timer;
            try {
                // Construct constraint matrix
                timer.reset();
                buildConstMtx();
                timer.stop();
                ROS_INFO_STREAM("RBPPlanner: Constraint Matrix runtime=" << timer.elapsedSeconds());

                // Solve QP
                timer.reset();
                solveQP(env, log);
                timer.stop();
                ROS_INFO_STREAM("RBPPlanner: QP runtime=" << timer.elapsedSeconds());
                ROS_INFO_STREAM("RBPPlanner: x size=" << count_x);
                ROS_INFO_STREAM("RBPPlanner: eq const size=" << count_eq);
                ROS_INFO_STREAM("RBPPlanner: ineq const size=" << count_lq);
            }
            catch (IloException &e) {
                ROS_ERROR_STREAM("RBPPlanner: CPLEX Concert exception caught: " << e);
                return false;
            }
            catch (...) {
                ROS_ERROR("RBPPlanner: CPLEX Unknown exception caught");
                return false;
            }
            env.end();

            if(param.time_scale) {
                timer.reset();
                timeScale();
                timer.stop();
                ROS_INFO_STREAM("RBPPlanner: timeScale runtime=" << timer.elapsedSeconds());
            }

            generateROSMsg();
            if(param.log){
                generateCoefCSV();
            }
            return true;
        }

    private:
        Mission mission;
        Param param;

        SwarmPlanning::PlanResult* planResult_ptr;
        std::vector<std::vector<int>> batches;
        int M, n, phi, N, outdim, offset_quad, offset_seg;
        IloNum count_x, count_eq, count_lq;

        // std::shared_ptr<Eigen::MatrixXd> Q_obj, Aeq_obj, Alq_obj, deq_obj, dlq_obj;
        Eigen::MatrixXd Q_base, Aeq_base, Alq, deq, dlq, basis;
        Eigen::MatrixXd dummy;
        std::vector<Eigen::MatrixXd> coef;

        void buildConstMtx() {
            build_Q_base();
            build_Aeq_base();
            build_deq();
            build_dlq();

            if (param.sequential) {
                build_dummy();
            }
        }

        void solveQP(const IloEnv &env, bool log) {
            Timer timer;
            IloNum total_cost = 0;

            IloCplex cplex(env);
//            cplex.setParam(IloCplex::Param::TimeLimit, 0.04);

            // publish Initial trajectory
            if(param.sequential && param.batch_iter == 0){
                // Translate Bernstein basis to Polynomial coefficients
                for (int k = 0; k < outdim; k++) {
                    for (int qi = 0; qi < N; qi++) {
                        for (int m = 0; m < M; m++) {
                            Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1, n + 1);
                            Eigen::MatrixXd tm;
                            timeMatrix(1.0 / (planResult_ptr->T[m+1] - planResult_ptr->T[m]), &tm);
                            tm = basis * tm;

                            for (int i = 0; i < n + 1; i++) {
                                c = c + dummy(qi * offset_quad + m * offset_seg + i, k) * tm.row(i);
                            }
                            coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
                        }
                    }
                    timer.stop();
                }
                return;
            }

            for (int iter = 0; iter < param.iteration; iter++) {
                total_cost = 0;
                for (int l = 0; l < param.batch_iter; l++) {
                    timer.reset();
                    IloModel model(env);
                    IloNumVarArray var(env);
                    IloRangeArray con(env);

                    populatebyrow(model, var, con, l);
                    cplex.extract(model);
                    if (log) {
                        std::string QPmodel_path = param.package_path + "/log/QPmodel.lp";
                        cplex.exportModel(QPmodel_path.c_str());
                    } else {
                        cplex.setOut(env.getNullStream());
                    }

                    // Optimize the problem and obtain solution.
                    if (!cplex.solve()) {
                        ROS_ERROR("RBPPlanner: Failed to optimize QP");
                        throw (-1);
                    }

                    IloNumArray vals(env);
                    total_cost += cplex.getObjValue();
                    cplex.getValues(vals, var);

                    // Translate Bernstein basis to Polynomial coefficients
                    int offset_dim = batches[l].size() * M * (n + 1);
                    int batch_max_iter = ceil((double)N / (double)param.batch_size);
                    for (int k = 0; k < outdim; k++) {
                        for (int qi = 0; qi < N; qi++) {
                            for (int m = 0; m < M; m++) {
                                Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1, n + 1);
                                Eigen::MatrixXd tm;
                                timeMatrix(1.0 / (planResult_ptr->T[m+1] - planResult_ptr->T[m]), &tm);
                                tm = basis * tm;

                                int bi = isQuadInBatch(qi, l);
                                if (bi >= 0) {
                                    for (int i = 0; i < n + 1; i++) {
                                        c = c + vals[k * offset_dim + bi * offset_quad + m * offset_seg + i] * tm.row(i);
                                        if (param.sequential) {
                                            dummy(qi * offset_quad + m * (n + 1) + i, k) = vals[k * offset_dim + bi * offset_quad + m * offset_seg + i];
                                        }
                                    }
                                    coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
                                } else if (param.sequential && param.batch_iter < batch_max_iter) {
                                    for (int i = 0; i < n + 1; i++) {
                                        c = c + dummy(qi * offset_quad + m * offset_seg + i, k) * tm.row(i);
                                    }
                                    coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
                                }
                            }
                        }
                        timer.stop();
                    }
                    if (param.sequential) {
                        ROS_INFO_STREAM("RBPPlanner: QP runtime of batch " << l << "=" << timer.elapsedSeconds());
                        ROS_INFO_STREAM("RBPPlanner: QP cost of batch " << l << "=" << cplex.getObjValue());
                    }
                }
                if (param.iteration > 1)
                    ROS_INFO_STREAM("RBPPlanner: QP iteration " << iter << " total_cost=" << total_cost);
            }
            ROS_INFO_STREAM("RBPPlanner: QP total cost=" << total_cost);
        }

        // For all segment of trajectory, check maximum velocity and accelation, and scale the segment time
        void timeScale() {
            if (phi != 3 || n != 5) {
                return;
            }

            Eigen::MatrixXd coef_der;
            double time_scale, time_scale_tmp, acc_max;
            time_scale = 1;
            for (int qi = 0; qi < N; qi++) {
                for (int k = 0; k < outdim; k++) {
                    for (int m = 0; m < M; m++) {
                        derivative_segment(qi, k, m, &coef_der);

                        time_scale_tmp = scale_to_max_vel(qi, k, m, coef_der);
                        if (time_scale < time_scale_tmp) {
                            time_scale = time_scale_tmp;
                        }

                        time_scale_tmp = scale_to_max_acc(qi, k, m, coef_der);
                        if (time_scale < time_scale_tmp) {
                            time_scale = time_scale_tmp;
                        }
                    }
                }
            }

            ROS_INFO_STREAM("RBPPlanner: Time scale=" << time_scale);
            if (time_scale != 1) {
                for (int qi = 0; qi < N; qi++) {
                    // trajectory
                    for (int k = 0; k < outdim; k++) {
                        for (int m = 0; m < M; m++) {
                            Eigen::MatrixXd tm;
                            timeMatrix(1.0 / time_scale, &tm);

                            coef[qi].block(m * offset_seg, k, n + 1, 1) =
                                    tm * coef[qi].block(m * offset_seg, k, n + 1, 1);
                        }
                    }

                    // SFC
                    for (int bi = 0; bi < planResult_ptr->SFC[qi].size(); bi++){
                        planResult_ptr->SFC[qi][bi].second *= time_scale;
                    }

                    // RSFC
                    for (int qj = qi + 1; qj < N; qj++){
                        for (int ri = 0; ri < planResult_ptr->RSFC[qi][qj].size(); ri++){
                            planResult_ptr->RSFC[qi][qj][ri].second *= time_scale;
                        }
                    }
                }
                // segment time
                for (int m = 0; m < M + 1; m++) {
                    planResult_ptr->T[m] *= time_scale;
                }
            }
        }

        // generate ros message to transfer planning result
        void generateROSMsg() {
            std::vector<double> traj_info;
            traj_info.emplace_back(N);
            traj_info.emplace_back(n);
            traj_info.insert(traj_info.end(), planResult_ptr->T.begin(), planResult_ptr->T.end());
            planResult_ptr->msgs_traj_info.data = traj_info;

            planResult_ptr->msgs_traj_coef.resize(N);
            for (int qi = 0; qi < N; qi++) {
                std_msgs::MultiArrayDimension rows;
                rows.size = M * (n + 1);
                planResult_ptr->msgs_traj_coef[qi].layout.dim.emplace_back(rows);

                std_msgs::MultiArrayDimension cols;
                cols.size = outdim;
                planResult_ptr->msgs_traj_coef[qi].layout.dim.emplace_back(cols);

                std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
                planResult_ptr->msgs_traj_coef[qi].data.insert(planResult_ptr->msgs_traj_coef[qi].data.end(),
                                                               coef_temp.begin(),
                                                               coef_temp.end());
            }
        }

        // translate coef to crazyswarm trajectory csv file
        // n should be smaller than 7
        void generateCoefCSV(){
            if(n > 7){
                ROS_WARN("RBPPlanner: n>8, do not make CSV file");
                return;
            }
            for(int qi = 0; qi < N; qi++) {
                std::ofstream coefCSV;
                coefCSV.open(param.package_path + "/log/coef" + std::to_string(qi + 1) + ".csv");
                coefCSV << "duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7\n";
                for(int m = 0; m < M; m++) {
                    coefCSV << planResult_ptr->T[m + 1] - planResult_ptr->T[m] << ",";
                    // x,y,z
                    for (int k = 0; k < outdim; k++) {
                        for (int i = n; i >= 0; i--) {
                            coefCSV << coef[qi](m * offset_seg + i, k) << ",";
                        }
                        // to match
                        for(int i = 0; i < 7 - n; i++){
                            coefCSV << "0,";
                        }
                    }
                    // yaw
                    for(int i = 0; i < 8; i++){
                        coefCSV << "0,";
                    }
                    coefCSV << "\n";
                }
                coefCSV.close();
            }
        }

        // Cost matrix Q
        void build_Q_base() {
            if (phi == 3 && n == 5) {
                Q_base = Eigen::MatrixXd::Zero(n + 1, n + 1);
                Q_base << 720, -1800, 1200, 0, 0, -120,
                        -1800, 4800, -3600, 0, 600, 0,
                        1200, -3600, 3600, -1200, 0, 0,
                        0, 0, -1200, 3600, -3600, 1200,
                        0, 600, 0, -3600, 4800, -1800,
                        -120, 0, 0, 1200, -1800, 720;

                basis = Eigen::MatrixXd::Zero(n + 1, n + 1);
                basis << -1, 5, -10, 10, -5, 1,
                        5, -20, 30, -20, 5, 0,
                        -10, 30, -30, 10, 0, 0,
                        10, -20, 10, 0, 0, 0,
                        -5, 5, 0, 0, 0, 0,
                        1, 0, 0, 0, 0, 0;
            } else {
                ROS_ERROR("RBPPlanner: n should be 5"); //TODO: debug when n is not 5
            }
        }

        Eigen::MatrixXd build_Q_p(int qi, int m) {
            return Q_base * pow(planResult_ptr->T[m+1] - planResult_ptr->T[m], -2 * phi + 1);
        }

        void build_Aeq_base() {
            Aeq_base = Eigen::MatrixXd::Zero((2 * phi + (M - 1) * phi), M * (n + 1));
            Eigen::MatrixXd A_waypoints = Eigen::MatrixXd::Zero(2 * phi, M * (n + 1));
            Eigen::MatrixXd A_cont = Eigen::MatrixXd::Zero((M - 1) * phi, M * (n + 1));
            Eigen::MatrixXd A_0 = Eigen::MatrixXd::Zero(n + 1, n + 1);
            Eigen::MatrixXd A_T = Eigen::MatrixXd::Zero(n + 1, n + 1);

            // Build A_0, A_T
            if (phi == 3 && n == 5) {
                A_0 << 1, 0, 0, 0, 0, 0,
                        -1, 1, 0, 0, 0, 0,
                        1, -2, 1, 0, 0, 0,
                        -1, 3, -3, 1, 0, 0,
                        1, -4, 6, -4, 1, 0,
                        -1, 5, -10, 10, -5, 1;

                A_T << 0, 0, 0, 0, 0, 1,
                        0, 0, 0, 0, -1, 1,
                        0, 0, 0, 1, -2, 1,
                        0, 0, -1, 3, -3, 1,
                        0, 1, -4, 6, -4, 1,
                        -1, 5, -10, 10, -5, 1;
            } else {
                ROS_ERROR("RBPPlanner: n should be 5"); //TODO: debug when n is not 5
            }

            // Build A_waypoints
            int nn = 1;
            for (int i = 0; i < phi; i++) {
                A_waypoints.block(i, 0, 1, n + 1) =
                        pow(planResult_ptr->T[1] - planResult_ptr->T[0], -i) * nn * A_0.row(i);
                A_waypoints.block(phi + i, (n + 1) * (M - 1), 1, n + 1) =
                        pow(planResult_ptr->T[planResult_ptr->T.size() - 1] - planResult_ptr->T[planResult_ptr->T.size() - 2], -i) * nn * A_T.row(i);
                nn = nn * (n - i);
            }

            // Build A_cont
            for (int m = 1; m < M; m++) {
                nn = 1;
                for (int j = 0; j < phi; j++) {
                    A_cont.block(phi * (m - 1) + j, (n + 1) * (m - 1), 1, n + 1) =
                            pow(planResult_ptr->T[m] - planResult_ptr->T[m-1], -j) * nn * A_T.row(j);
                    A_cont.block(phi * (m - 1) + j, (n + 1) * m, 1, n + 1) =
                            -pow(planResult_ptr->T[m+1] - planResult_ptr->T[m], -j) * nn * A_0.row(j);
                    nn = nn * (n - j);
                }
            }

            // Build Aeq_base
            Aeq_base << A_waypoints,
                    A_cont;

        }

        // Equality constraints condition vector deq
        void build_deq() {
//        deq_obj.reset(new Eigen::MatrixXd(N * (2*n + (M-1)*n), outdim));
//        deq_obj->setZero();
            deq = Eigen::MatrixXd::Zero(N * (2 * phi + (M - 1) * phi), outdim);
            for (int qi = 0; qi < N; qi++) {
                Eigen::MatrixXd d_waypoints = Eigen::MatrixXd::Zero(2 * phi, outdim);
                Eigen::MatrixXd d_cont = Eigen::MatrixXd::Zero((M - 1) * phi, outdim);
                for (int k = 0; k < outdim; k++) {
                    d_waypoints(0, k) = mission.startState[qi][k];
                    d_waypoints(1, k) = mission.startState[qi][k + 3];
                    d_waypoints(2, k) = mission.startState[qi][k + 6];
                    d_waypoints(phi, k) = mission.goalState[qi][k];
                    d_waypoints(phi + 1, k) = mission.goalState[qi][k + 3];
                    d_waypoints(phi + 2, k) = mission.goalState[qi][k + 6];
                }

                // Build deq
                int deq_p_rows = d_waypoints.rows() + d_cont.rows();
                int deq_p_cols = outdim;
//            deq_obj->block(qi * deq_p_rows, 0, deq_p_rows, deq_p_cols) << d_waypoints,
//                                                                          d_cont;
                deq.block(qi * deq_p_rows, 0, deq_p_rows, deq_p_cols) << d_waypoints,
                        d_cont;
            }
        }

        // Inequality constraints condition vector dlq
        void build_dlq() {
//        dlq_obj.reset(new Eigen::MatrixXd(N*2*(n+1)*M + N*(N-1)*(n+1)*M, outdim));
//        dlq_obj->setZero();
            dlq = Eigen::MatrixXd::Zero(N * 2 * (n + 1) * M + N * (N - 1) * (n + 1) * M, outdim);
            Eigen::MatrixXd dlq_rel = Eigen::MatrixXd::Zero(N * (N - 1) * (n + 1) * M, outdim);
            Eigen::MatrixXd dlq_box = Eigen::MatrixXd::Zero(N * 2 * (n + 1) * M, outdim);

            // Build dlq_box
            for (int qi = 0; qi < N; qi++) {
                Eigen::MatrixXd d_upper = Eigen::MatrixXd::Zero((n + 1) * M, outdim);
                Eigen::MatrixXd d_lower = Eigen::MatrixXd::Zero((n + 1) * M, outdim);

                int bi = 0;
                for (int m = 0; m < M; m++) {
                    // find box number
                    while (bi < planResult_ptr->SFC[qi].size() &&
                           planResult_ptr->SFC[qi][bi].second < planResult_ptr->T[m + 1]) {
                        bi++;
                    }

                    d_upper.block((n + 1) * m, 0, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, planResult_ptr->SFC[qi][bi].first[3]);
                    d_lower.block((n + 1) * m, 0, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, -planResult_ptr->SFC[qi][bi].first[0]);

                    d_upper.block((n + 1) * m, 1, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, planResult_ptr->SFC[qi][bi].first[4]);
                    d_lower.block((n + 1) * m, 1, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, -planResult_ptr->SFC[qi][bi].first[1]);

                    d_upper.block((n + 1) * m, 2, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, planResult_ptr->SFC[qi][bi].first[5]);
                    d_lower.block((n + 1) * m, 2, n + 1, 1) =
                            Eigen::MatrixXd::Constant(n + 1, 1, -planResult_ptr->SFC[qi][bi].first[2]);
                }

                int dlq_box_rows = d_upper.rows() + d_lower.rows();
                dlq_box.block(qi * dlq_box_rows, 0, dlq_box_rows, outdim) << d_upper,
                        d_lower;
            }

            // Build dlq_rel
            int iter = 0;
            for (int qi = 0; qi < N; qi++) {
                for (int qj = qi + 1; qj < N; qj++) {
                    Eigen::MatrixXd d_upper = Eigen::MatrixXd::Constant((n + 1) * M, outdim, 10000000);
                    Eigen::MatrixXd d_lower = Eigen::MatrixXd::Constant((n + 1) * M, outdim, 10000000);

                    for (int m = 0; m < M; m++) {
                        // Find box number
                        int ri = 0;
                        while (ri < planResult_ptr->RSFC[qi][qj].size() &&
                               planResult_ptr->RSFC[qi][qj][ri].second < planResult_ptr->T[m + 1]) {
                            ri++;
                        }

                        octomap::point3d normal_vector = planResult_ptr->RSFC[qi][qj][ri].first;
                        d_upper.block((n + 1) * m, 0, n + 1, 1) =
                                Eigen::MatrixXd::Constant(n + 1, 1, normal_vector.x());
                        d_upper.block((n + 1) * m, 1, n + 1, 1) =
                                Eigen::MatrixXd::Constant(n + 1, 1, normal_vector.y());
                        d_upper.block((n + 1) * m, 2, n + 1, 1) =
                                Eigen::MatrixXd::Constant(n + 1, 1, normal_vector.z());
                    }
                    int dlq_rel_rows = d_upper.rows() + d_lower.rows();
                    dlq_rel.block(iter * dlq_rel_rows, 0, dlq_rel_rows, outdim) << d_upper,
                            d_lower;
                    iter++;
                }
            }

            // Build dlq
//        dlq_obj->block(0, 0, dlq_box.rows(), dlq_box.cols()) = dlq_box;
//        dlq_obj->block(dlq_box.rows(), 0, dlq_rel.rows(), dlq_rel.cols()) = dlq_rel;
            dlq << dlq_box,
                    dlq_rel;
        }

        void build_dummy() {
            dummy = Eigen::MatrixXd::Zero(N * offset_quad, outdim);

            for (int qi = 0; qi < N; qi++) {
                int m = 0;
                int idx = 0;
                while (m < M) {
                    if (idx >= planResult_ptr->initTraj[qi].size() - 1) {
                        idx = planResult_ptr->initTraj[qi].size() - 1;
                        for (int j = 0; j < n + 1; j++) {
                            dummy(qi * offset_quad + m * (n + 1) + j, 0) = planResult_ptr->initTraj[qi][idx].x();
                            dummy(qi * offset_quad + m * (n + 1) + j, 1) = planResult_ptr->initTraj[qi][idx].y();
                            dummy(qi * offset_quad + m * (n + 1) + j, 2) = planResult_ptr->initTraj[qi][idx].z();
                        }
                        m++;
                    } else {
                        for (int j = 0; j < n + 1; j++) {
                            int a = 1;
                            if (j < (n + 1) / 2) {
                                a = 0;
                            }
                            dummy(qi * offset_quad + m * (n + 1) + j, 0) =
                                    (1 - a) * planResult_ptr->initTraj[qi][idx].x()
                                    + a * planResult_ptr->initTraj[qi][idx + 1].x();
                            dummy(qi * offset_quad + m * (n + 1) + j, 1) =
                                    (1 - a) * planResult_ptr->initTraj[qi][idx].y()
                                    + a * planResult_ptr->initTraj[qi][idx + 1].y();
                            dummy(qi * offset_quad + m * (n + 1) + j, 2) =
                                    (1 - a) * planResult_ptr->initTraj[qi][idx].z()
                                    + a * planResult_ptr->initTraj[qi][idx + 1].z();
                        }
                        m++;
                    }
                    idx++;
                }
            }
        }

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c, int l) {
            int offset_dim = batches[l].size() * M * (n + 1);
            IloEnv env = model.getEnv();
            for (int k = 0; k < outdim; k++) {
                for (int bi = 0; bi < batches[l].size(); bi++) {
                    for (int m = 0; m < M; m++) {
                        for (int i = 0; i < n + 1; i++) {
                            x.add(IloNumVar(env, -IloInfinity, IloInfinity));

                            int qi = batches[l][bi];
                            int row = k * offset_dim + bi * offset_quad + m * (n + 1) + i;
                            std::string name;
                            if (k == 0) {
                                name = "x_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                            } else if (k == 1) {
                                name = "y_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                            } else if (k == 2) {
                                name = "z_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                            } else {
                                ROS_ERROR("RBPPlanner: Invalid outdim");
                            }

                            x[row].setName(name.c_str());
                        }
                    }
                }
            }
            count_x = x.getSize();

            // Cost function
            IloNumExpr cost(env);
            for (int k = 0; k < outdim; k++) {
                for (int bi = 0; bi < batches[l].size(); bi++) {
                    for (int m = 0; m < M; m++) {
                        int qi = batches[l][bi];
                        Eigen::MatrixXd Q_p = build_Q_p(qi, m);

                        for (int i = 0; i < n + 1; i++) {
                            int row = qi * M * (n + 1) + m * (n + 1) + i;
                            int row_idx = k * offset_dim + bi * offset_quad + m * (n + 1) + i;

                            for (int j = 0; j < n + 1; j++) {
                                int col = qi * M * (n + 1) + m * (n + 1) + j;
                                int col_idx =
                                        k * offset_dim + bi * offset_quad + m * (n + 1) + j;

                                if (Q_p(i, j) != 0) {
                                    cost += Q_p(i, j) * x[row_idx] * x[col_idx];
                                }
                            }
                        }
                    }
                }
            }
            model.add(IloMinimize(env, cost));

            // Equality Constraints
            for (int k = 0; k < outdim; k++) {
                for (int bi = 0; bi < batches[l].size(); bi++) {
                    for (int i = 0; i < 2 * phi + (M - 1) * phi; i++) {
                        IloNumExpr expr(env);
                        for (int j = 0; j < M * (n + 1); j++) {
                            if (Aeq_base(i, j) != 0) {
                                expr += Aeq_base(i, j) * x[k * offset_dim + bi * offset_quad + j];
                            }
                        }
                        int qi = batches[l][bi];
                        c.add(expr == deq(qi * (2 * phi + (M - 1) * phi) + i, k));
                        expr.end();
                    }
                }
            }
            count_eq = c.getSize();

            // Inequality Constraints
            for (int k = 0; k < outdim; k++) {
                for (int bi = 0; bi < batches[l].size(); bi++) {
                    int qi = batches[l][bi];
                    for (int j = 0; j < (n + 1) * M; j++) {
                        int idx = k * offset_dim + bi * offset_quad + j;
                        c.add(x[idx] <= dlq(2 * qi * offset_quad + j, k));
                        c.add(-x[idx] <= dlq((2 * qi + 1) * offset_quad + j, k));
                    }
                }
            }
            int offset_box = 2 * N * offset_quad;
            int iter = 0;
            for (int qi = 0; qi < N; qi++) {
                for (int qj = qi + 1; qj < N; qj++) {
                    int bi = isQuadInBatch(qi, l);
                    int bj = isQuadInBatch(qj, l);

                    if (bi < 0 && bj < 0) {

                    } else if (bi >= 0 && bj < 0) {
                        for (int j = 0; j < M * (n + 1); j++) {
                            int idx = bi * offset_quad + j;
                            c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) *
                                  (dummy(qj * offset_quad + j, 0) - x[0 * offset_dim + idx]) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 1) *
                                  (dummy(qj * offset_quad + j, 1) - x[1 * offset_dim + idx]) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 2) *
                                  (dummy(qj * offset_quad + j, 2) - x[2 * offset_dim + idx])
                                  >= mission.quad_size[qi] + mission.quad_size[qj]);
                        }
                    } else if (bi < 0 && bj >= 0) {
                        for (int j = 0; j < M * (n + 1); j++) {
                            int jdx = bj * offset_quad + j;
                            c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) *
                                  (x[0 * offset_dim + jdx] - dummy(qi * offset_quad + j, 0)) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 1) *
                                  (x[1 * offset_dim + jdx] - dummy(qi * offset_quad + j, 1)) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 2) *
                                  (x[2 * offset_dim + jdx] - dummy(qi * offset_quad + j, 2))
                                  >= mission.quad_size[qi] + mission.quad_size[qj]);
                        }
                    } else {
                        for (int j = 0; j < M * (n + 1); j++) {
                            int idx = bi * offset_quad + j;
                            int jdx = bj * offset_quad + j;

                            c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) *
                                  (x[0 * offset_dim + jdx] - x[0 * offset_dim + idx]) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 1) *
                                  (x[1 * offset_dim + jdx] - x[1 * offset_dim + idx]) +
                                  dlq(offset_box + offset_quad * 2 * iter + j, 2) *
                                  (x[2 * offset_dim + jdx] - x[2 * offset_dim + idx])
                                  >= mission.quad_size[qi] + mission.quad_size[qj]);
                        }
                    }

                    iter++;
                }
            }
            model.add(c);

            count_lq = c.getSize() - count_eq;
        }

        // timeMatrix is mapping matrix (n + 1) x (n + 1)
        // e.g. [1   0   0   ...]
        //      [0   t   0   ...]
        //      [0   0   t^2 ...]
        //      [... ... ... ...]
        void timeMatrix(double t, Eigen::MatrixXd* tm_ptr) {
            *tm_ptr = Eigen::MatrixXd::Zero(n + 1, n + 1);
            for (int i = 0; i < n + 1; i++) {
                (*tm_ptr)(i, i) = pow(t, n - i);
            }
        }

        // Get derivative of m^th segment(polynomial) of qi^th agent
        // e.g. if phi = 3, n = 5, polynomial [1 1 1 1 1 1] then
        //      coef_der = [1  1  1  1  1  1]
        //                 [5  4  3  2  1  0]
        //                 [20 12 6  2  0  0]
        //                 [60 24 6  0  0  0]
        void derivative_segment(int qi, int k, int m, Eigen::MatrixXd* coef_der_ptr) {
            *coef_der_ptr = Eigen::MatrixXd::Zero(phi + 1, n + 1);
            for (int i = 0; i < phi + 1; i++) {
                for (int j = 0; j < n + 1; j++) {
                    if (i <= j)
                        (*coef_der_ptr)(i, n - j) = coef_derivative(i, j) * coef[qi](m * offset_seg + n - j, k);
                    else
                        (*coef_der_ptr)(i, n - j) = 0;
                }
            }
        }

        // Get j^th coefficient of i^th derivative of polynomial [1 1 1 1 ... 1]
        int coef_derivative(int i, int j) {
            return (i == 0) ? 1 : coef_derivative(i - 1, j - 1) * j;
        }

        // Get roots of i^th derivative of polynomial with coefficient coef
        // The roots of the polynomial are calculated by computing the eigenvalues of the companion matrix, A
        std::vector<double> roots_derivative(int i, const Eigen::MatrixXd& coef_der) {
            std::vector<double> roots_der;
            int n_der = n - i;
            int iter = 0;
            while(n_der > 0 && coef_der(i, n - i - n_der) == 0 ){
                n_der--;
            }
            if(n_der == 0){
                return roots_der; // return empty vector
            }

            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_der, n_der);
            for (int j = 0; j < n_der; j++) {
                if (j < n_der - 1) {
                    A(j + 1, j) = 1;
                }
                A(0, j) = -coef_der(i, n - i - n_der + j + 1) / coef_der(i, n - i - n_der);
            }

            Eigen::EigenSolver<Eigen::MatrixXd> es(A);
            for (int j = 0; j < i; j++) {
                complex<double> lambda = es.eigenvalues()[j];
                if (lambda.imag() == 0) {
                    roots_der.emplace_back(lambda.real());
                }
            }
            return roots_der;
        }

        double scale_to_max_vel(int qi, int k, int m, const Eigen::MatrixXd& coef_der) {
            assert(phi > 1);
            double scale_update_rate = 1.1; //TODO: parameterization?

            // Get maximum velocity
            double vel_max = 0, t_max = 0;
            std::vector<double> ts = roots_derivative(2, coef_der);
            ts.emplace_back(0);
            ts.emplace_back(planResult_ptr->T[m + 1] - planResult_ptr->T[m]);
            for (auto t : ts) {
                if (t < 0 || t > planResult_ptr->T[m + 1] - planResult_ptr->T[m]) {
                    continue;
                }

                double vel = 0;
                for (int i = 0; i < n - 1; i++) {
                    vel += coef_der(1, i) * pow(t, n - 1 - i);
                }
                vel = abs(vel);
                if (vel_max < vel) {
                    vel_max = vel;
                    t_max = t;
                }
            }

            // time_scale update
            double time_scale = 1;
            while (vel_max > mission.max_vel[qi][k]) {
                time_scale *= scale_update_rate;

                double vel = 0;
                for (int i = 0; i < n - 1; i++) {
                    vel += coef_der(1, i) * pow(1 / time_scale, n - i) * pow(t_max, n - 1 - i);
                }
                vel_max = abs(vel);
            }

            return time_scale;
        }


        double scale_to_max_acc(int qi, int k, int m, const Eigen::MatrixXd &coef_der) {
            assert(phi == 3 && n == 5);
            double scale_update_rate = 1.1; //TODO: parameterization?

            // Get maximum accelaration
            double a, b, c, D, acc_max, t_max = 0;
            a = coef_der(3, 0);
            b = coef_der(3, 1);
            c = coef_der(3, 2);
            D = b * b - 4 * a * c;
            acc_max = 0;

            std::vector<double> ts{0, planResult_ptr->T[m+1] - planResult_ptr->T[m]};
            if (D >= 0 && a != 0) {
                ts.emplace_back((-b + sqrt(D)) / (2 * a));
                ts.emplace_back((-b - sqrt(D)) / (2 * a));
            }
            else if(a == 0 && b != 0){
                ts.emplace_back(-c/b);
            }

            for (auto t : ts) {
                if (t < 0 || t > planResult_ptr->T[m+1] - planResult_ptr->T[m]) {
                    continue;
                }

                double acc = 0;
                for (int i = 0; i < 4; i++) {
                    acc += coef_der(2, i) * pow(t, 3 - i);
                }
                acc = abs(acc);
                if (acc_max < acc) {
                    acc_max = acc;
                    t_max = t;
                }
            }

            // time_scale update
            double time_scale = 1;
            while (acc_max > mission.max_acc[qi][k]) {
                time_scale *= scale_update_rate;

                double acc = 0;
                for (int i = 0; i < 4; i++) {
                    acc += coef_der(2, i) * pow(1 / time_scale, n - i) * pow(t_max, 3 - i);
                }
                acc_max = abs(acc);
            }

            return time_scale;
        }

        void setBatch(int alg){
            int batch_max_iter = ceil((double)N / (double)param.batch_size);
            if (param.sequential) {
                int batch_max_iter = ceil((double)N / (double)param.batch_size);
                if (param.batch_iter < 0 || param.batch_iter > batch_max_iter) {
                    param.batch_iter = batch_max_iter;
                }
            } else {
                param.batch_size = N;
                param.batch_iter = 1;
            }

            batches.resize(batch_max_iter);

            //default groups
            if(alg == 0) {
                for (int qi = 0; qi < N; qi++) {
                    batches[qi / param.batch_size].emplace_back(qi);
                }
            }
            else{
                ROS_ERROR("RBPPlaner: invalid batch algorithm");
            }
        }

        int isQuadInBatch(int qi, int l){
            for(int bi = 0; bi < batches[l].size(); bi++){
                if(qi == batches[l][bi]){
                    return bi;
                }
            }
            return -1;
        }
    };
}