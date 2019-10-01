#pragma once

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

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

class RBPPlanner {
public:
    std_msgs::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;

    RBPPlanner(std::shared_ptr<Corridor> _corridor_obj,
               std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
               SwarmPlanning::Mission _mission,
               SwarmPlanning::Param _param)
            : corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))

    {
        M = initTrajPlanner_obj.get()->T.size()-1; // the number of segments
        n = param.n; // degree of polynomial
        phi = param.phi; // desired derivatives
        N = mission.qn; // the number of agents
        outdim = 3; // the number of outputs (x,y,z)

        initTraj = initTrajPlanner_obj.get()->initTraj;
        T = initTrajPlanner_obj.get()->T;

        SFC = corridor_obj.get()->SFC;
        RSFC = corridor_obj.get()->RSFC;

        if(param.sequential){
            if(param.N_b > 0 && param.N_b < N / param.batch_size){

            }
            else{
                param.N_b = N / param.batch_size; //TODO: the number of batch, NOTE THAT N/batch_size should be integer
            }
        }
        else{
            param.batch_size = N;
            param.N_b = 1;
        }

        //N_b = std::ceil(static_cast<double>(N) / static_cast<double>(plan_batch_size));

        offset_dim = param.batch_size * M * (n + 1);
        offset_quad = M * (n + 1);
        offset_seg = n + 1;
    }

    bool update(bool log){
        IloEnv env;
        Timer timer;

        coef.resize(N);
        for(int qi = 0; qi < N; qi++) {
            coef[qi] = Eigen::MatrixXd::Zero(offset_quad, outdim);
        }
        try {
            timer.reset();
            buildConstMtx();
            timer.stop();
            ROS_INFO_STREAM("Constraint Matrix runtime: " << timer.elapsedSeconds());

            timer.reset();
            solveQP(env, log);
            timer.stop();
            ROS_INFO_STREAM("QP runtime: " << timer.elapsedSeconds());
            ROS_INFO_STREAM("x size: " << count_x);
            ROS_INFO_STREAM("eq const size: " << count_eq);
            ROS_INFO_STREAM("ineq const size: " << count_lq);

        }
        catch (IloException& e) {
            ROS_ERROR_STREAM("Concert exception caught: " << e);
            return false;
        }
        catch (...) {
            ROS_ERROR("Unknown exception caught");
            return false;
        }
        env.end();

        timeScale();
        createMsg();
        return true;
    }

private:
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    SFC_t SFC;
    RSFC_t RSFC;

    int M, n, phi, N, outdim, offset_dim, offset_quad, offset_seg;
    IloNum count_x, count_eq, count_lq;

    // std::shared_ptr<Eigen::MatrixXd> Q_obj, Aeq_obj, Alq_obj, deq_obj, dlq_obj;
    Eigen::MatrixXd Q_base, Aeq_base, Alq, deq, dlq, basis;
    Eigen::MatrixXd dummy;
    std::vector<Eigen::MatrixXd> coef;

    void buildConstMtx(){
        build_Q_base();
        build_Aeq_base();
        build_deq();
        build_dlq();

        if(param.sequential){
            build_dummy();
        }
//        build_dummy();

    }

    void solveQP(const IloEnv& env, bool log){
        Timer timer;
        IloNum total_cost = 0;

        IloCplex cplex(env);
//        cplex.setParam(IloCplex::Param::TimeLimit, 0.04);

        for(int iter = 0; iter < param.iteration; iter++) {
            total_cost = 0;
            for (int gi = 0; gi < param.N_b; gi++) {
                timer.reset();
                IloModel model(env);
                IloNumVarArray var(env);
                IloRangeArray con(env);

                populatebyrow(model, var, con, gi);

                cplex.extract(model);

                if (log) {
                    std::string QPmodel_path = param.package_path + "/log/QPmodel.lp";
                    cplex.exportModel(QPmodel_path.c_str());
                } else {
                    cplex.setOut(env.getNullStream());
                }

                // Optimize the problem and obtain solution.
                if (!cplex.solve()) {
                    env.error() << "Failed to optimize QP, Check ~/.ros/QPresult.txt" << endl;
                    throw (-1);
                }

                IloNumArray vals(env);
                total_cost += cplex.getObjValue();
                cplex.getValues(vals, var);

                // Translate Bernstein basis to Polynomial coefficients
                for (int k = 0; k < outdim; k++) {
                    for (int qi = 0; qi < N; qi++) {
                        for (int m = 0; m < M; m++) {
                            Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1, n + 1);
                            Eigen::MatrixXd tm;
                            timeMatrix(1.0 / (T[m + 1] - T[m]), tm);
                            tm = basis * tm;

                            if (qi >= gi * param.batch_size && qi < (gi + 1) * param.batch_size) {
                                for (int i = 0; i < n + 1; i++) {
                                    c = c +
                                        vals[k * offset_dim + (qi - gi * param.batch_size) * offset_quad + m * offset_seg + i] *
                                        tm.row(i);
                                    if (param.sequential) {
                                        dummy(qi * offset_quad + m * (n + 1) + i, k) =
                                                vals[k * offset_dim + (qi - gi * param.batch_size) * offset_quad +
                                                     m * offset_seg + i];
                                    }
                                }
                                coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
                            } else if (param.sequential && param.N_b < N / param.batch_size) {
                                for (int i = 0; i < n + 1; i++) {
                                    c = c + dummy(qi * offset_quad + m * offset_seg + i, k) * tm.row(i);
                                }
                                coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
                            }
//                            {
//                                for (int i = 0; i < n + 1; i++) {
//                                    c = c + dummy(qi * offset_quad + m * offset_seg + i, k) * tm.row(i);
//                                }
//                                coef[qi].block(m * offset_seg, k, n + 1, 1) = c.transpose();
//                            }
                        }
                    }
                    timer.stop();
                }
                if (param.sequential) {
                    ROS_INFO_STREAM("QP runtime of batch " << gi << ": " << timer.elapsedSeconds());
                    ROS_INFO_STREAM("QP cost of batch " << gi << ": " << cplex.getObjValue());
                }
            }
            if(param.iteration > 1)
                ROS_INFO_STREAM("QP iteration " << iter << " total_cost: " << total_cost);
        }
        ROS_INFO_STREAM("QP total cost: " << total_cost);
    }

    void timeScale(){
        if(phi != 3 || n != 5){
            return;
        }

        Eigen::MatrixXd coef_der;
        double time_scale, time_scale_tmp, acc_max;
        time_scale = 1;
        for(int qi = 0; qi < N; qi++){
            for(int k = 0; k < outdim; k++) {
                for (int m = 0; m < M; m++) {
                    derivative(qi, k, m, coef_der); //TODO: coef_def explanation

                    time_scale_tmp = scale_to_max_vel(qi, k, m, coef_der);
                    if(time_scale < time_scale_tmp) {
                        time_scale = time_scale_tmp;
                    }

                    time_scale_tmp = scale_to_max_acc(qi, k, m, coef_der);
                    if(time_scale < time_scale_tmp) {
                        time_scale = time_scale_tmp;
                    }
                }
            }
        }

        ROS_INFO_STREAM("Time scale: " << time_scale);
        if(time_scale != 1){
            for (int k = 0; k < outdim; k++) {
                for (int qi = 0; qi < N; qi++) {
                    for (int m = 0; m < M; m++) {
                        Eigen::MatrixXd tm;
                        timeMatrix(1.0 / time_scale, tm);

                        coef[qi].block(m * offset_seg, k, n + 1, 1) = tm * coef[qi].block(m * offset_seg, k, n + 1, 1);
                    }
                }
            }
            for (int m = 0; m < M+1; m++) {
                T[m] = T[m] * time_scale;
            }
        }
    }

    void createMsg(){
        std::vector<double> traj_info;
        traj_info.emplace_back(N);
        traj_info.emplace_back(n);
        traj_info.insert(traj_info.end(), T.begin(), T.end());
        msgs_traj_info.data = traj_info;

        msgs_traj_coef.resize(N);
        for(int qi = 0; qi < N; qi++) {
            std_msgs::MultiArrayDimension rows;
            rows.size = M * (n + 1);
            msgs_traj_coef[qi].layout.dim.emplace_back(rows);

            std_msgs::MultiArrayDimension cols;
            cols.size = outdim;
            msgs_traj_coef[qi].layout.dim.emplace_back(cols);

            std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
            msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
        }
    }

    // Cost matrix Q
    void build_Q_base() {
        if (phi == 3 && n == 5) {
            Q_base = Eigen::MatrixXd::Zero(n+1, n+1);
            Q_base <<   720, -1800,  1200,     0,     0,  -120,
                      -1800,  4800, -3600,     0,   600,     0,
                       1200, -3600,  3600, -1200,     0,     0,
                          0,     0, -1200,  3600, -3600,  1200,
                          0,   600,     0, -3600,  4800, -1800,
                       -120,     0,     0,  1200, -1800,   720;

            basis = Eigen::MatrixXd::Zero(n+1, n+1);
            basis <<   -1,     5,   -10,    10,    -5,     1,
                        5,   -20,    30,   -20,     5,     0,
                      -10,    30,   -30,    10,     0,     0,
                       10,   -20,    10,     0,     0,     0,
                       -5,     5,     0,     0,     0,     0,
                        1,     0,     0,     0,     0,     0;
        } else {
            std::cerr << "TODO: debug when n is not 5" << std::endl; //TODO: debug when n is not 5
        }
    }

    Eigen::MatrixXd build_Q_p(int qi, int m) {
        return Q_base * pow(T[m + 1] - T[m], -2*phi+1);
    }

    void build_Aeq_base() {
        Aeq_base = Eigen::MatrixXd::Zero((2*phi + (M-1)*phi), M*(n+1));
        Eigen::MatrixXd A_waypoints = Eigen::MatrixXd::Zero(2*phi, M*(n+1));
        Eigen::MatrixXd A_cont = Eigen::MatrixXd::Zero((M-1)*phi, M*(n+1));
        Eigen::MatrixXd A_0 = Eigen::MatrixXd::Zero(n+1, n+1);
        Eigen::MatrixXd A_T = Eigen::MatrixXd::Zero(n+1, n+1);

        // Build A_0, A_T
        if (phi == 3 && n == 5) {
            A_0 <<   1,  0,  0,  0,  0,  0,
                    -1,  1,  0,  0,  0,  0,
                     1, -2,  1,  0,  0,  0,
                    -1,  3, -3,  1,  0,  0,
                     1, -4,  6, -4,  1,  0,
                    -1,  5,-10, 10, -5,  1;

            A_T <<   0,  0,  0,  0,  0,  1,
                     0,  0,  0,  0, -1,  1,
                     0,  0,  0,  1, -2,  1,
                     0,  0, -1,  3, -3,  1,
                     0,  1, -4,  6, -4,  1,
                    -1,  5,-10, 10, -5,  1;
        } else {
            ROS_ERROR("TODO: debug when n is not 5"); //TODO: debug when n is not 5
        }

        // Build A_waypoints
        int nn = 1;
        for (int i = 0; i < phi; i++) {
            A_waypoints.block(i, 0, 1, n+1) = pow(T[1] - T[0], -i) * nn * A_0.row(i);
            A_waypoints.block(phi+i, (n+1)*(M-1), 1, n+1) =
                    pow(T[T.size()-1]-T[T.size()-2], -i) * nn * A_T.row(i);
            nn = nn * (n-i);
        }

        // Build A_cont
        for (int m = 1; m < M; m++) {
            nn = 1;
            for (int j = 0; j < phi; j++) {
                A_cont.block(phi*(m-1)+j, (n+1)*(m-1), 1, n+1) = pow(T[m] - T[m-1], -j) * nn * A_T.row(j);
                A_cont.block(phi*(m-1)+j, (n+1)*m, 1, n+1) = -pow(T[m+1] - T[m], -j) * nn * A_0.row(j);
                nn = nn * (n-j);
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
        deq = Eigen::MatrixXd::Zero(N * (2*phi + (M-1)*phi), outdim);
        for (int qi = 0; qi < N; qi++) {
            Eigen::MatrixXd d_waypoints = Eigen::MatrixXd::Zero(2 * phi, outdim);
            Eigen::MatrixXd d_cont = Eigen::MatrixXd::Zero((M - 1) * phi, outdim);
            for (int k = 0; k < outdim; k++) {
                d_waypoints(0, k) = mission.startState[qi][k];
                d_waypoints(1, k) = mission.startState[qi][k+3];
                d_waypoints(2, k) = mission.startState[qi][k+6];
                d_waypoints(phi, k) = mission.goalState[qi][k];
                d_waypoints(phi+1, k) = mission.goalState[qi][k+3];
                d_waypoints(phi+2, k) = mission.goalState[qi][k+6];
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
    void build_dlq(){
//        dlq_obj.reset(new Eigen::MatrixXd(N*2*(n+1)*M + N*(N-1)*(n+1)*M, outdim));
//        dlq_obj->setZero();
        dlq = Eigen::MatrixXd::Zero(N*2*(n+1)*M + N*(N-1)*(n+1)*M, outdim);
        Eigen::MatrixXd dlq_rel = Eigen::MatrixXd::Zero(N*(N-1)*(n+1)*M, outdim);
        Eigen::MatrixXd dlq_box = Eigen::MatrixXd::Zero(N*2*(n+1)*M, outdim);

        // Build dlq_box
        for(int qi=0; qi<N; qi++){
            Eigen::MatrixXd d_upper = Eigen::MatrixXd::Zero((n+1)*M, outdim);
            Eigen::MatrixXd d_lower = Eigen::MatrixXd::Zero((n+1)*M, outdim);

            int bi = 0;
            for(int m = 0; m < M; m++){
                // find box number
                while(bi < SFC[qi].size() && SFC[qi][bi].second < T[m+1]){
                    bi++;
                }

                d_upper.block((n+1)*m, 0, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, SFC[qi][bi].first[3]);
                d_lower.block((n+1)*m, 0, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, -SFC[qi][bi].first[0]);

                d_upper.block((n+1)*m, 1, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, SFC[qi][bi].first[4]);
                d_lower.block((n+1)*m, 1, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, -SFC[qi][bi].first[1]);

                d_upper.block((n+1)*m, 2, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, SFC[qi][bi].first[5]);
                d_lower.block((n+1)*m, 2, n+1, 1) =
                        Eigen::MatrixXd::Constant(n+1, 1, -SFC[qi][bi].first[2]);
            }

            int dlq_box_rows = d_upper.rows() + d_lower.rows();
            dlq_box.block(qi*dlq_box_rows, 0, dlq_box_rows, outdim) << d_upper,
                    d_lower;
        }

        // Build dlq_rel
        int iter = 0;
        for(int qi = 0; qi < N; qi++){
            for(int qj = qi + 1; qj < N; qj++){
                Eigen::MatrixXd d_upper = Eigen::MatrixXd::Constant((n+1)*M, outdim, 10000000);
                Eigen::MatrixXd d_lower = Eigen::MatrixXd::Constant((n+1)*M, outdim, 10000000);

                for(int m = 0; m < M; m++){
                    // Find box number
                    int ri = 0;
                    while(ri < RSFC[qi][qj].size() && RSFC[qi][qj][ri].second < T[m+1]){
                        ri++;
                    }

                    octomap::point3d normal_vector = RSFC[qi][qj][ri].first;
                    d_upper.block((n+1)*m, 0, n+1, 1) =
                            Eigen::MatrixXd::Constant(n+1, 1, normal_vector.x());
                    d_upper.block((n+1)*m, 1, n+1, 1) =
                            Eigen::MatrixXd::Constant(n+1, 1, normal_vector.y());
                    d_upper.block((n+1)*m, 2, n+1, 1) =
                            Eigen::MatrixXd::Constant(n+1, 1, normal_vector.z());
                }
                int dlq_rel_rows = d_upper.rows() + d_lower.rows();
                dlq_rel.block(iter*dlq_rel_rows, 0, dlq_rel_rows, outdim) << d_upper,
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

    void build_dummy(){
        dummy = Eigen::MatrixXd::Zero(N * offset_quad, outdim);

        for(int qi = 0; qi < N; qi++){
            int m = 0;
            int idx = 0;
            while(m < M){
                if(idx >= initTraj[qi].size() - 1){
                    idx = initTraj[qi].size() - 1;
                    for(int j = 0; j < n+1; j++) {
                        dummy(qi * offset_quad + m * (n+1) + j, 0) = initTraj[qi][idx].x();
                        dummy(qi * offset_quad + m * (n+1) + j, 1) = initTraj[qi][idx].y();
                        dummy(qi * offset_quad + m * (n+1) + j, 2) = initTraj[qi][idx].z();
                    }
                    m++;
                }
                else{
                    for(int j = 0; j < n+1; j++) {
                        int a = 1;
                        if(j < (n+1)/2){
                            a = 0;
                        }
                        dummy(qi * offset_quad + m * (n+1) + j, 0) = (1-a) * initTraj[qi][idx].x() + a * initTraj[qi][idx+1].x();
                        dummy(qi * offset_quad + m * (n+1) + j, 1) = (1-a) * initTraj[qi][idx].y() + a * initTraj[qi][idx+1].y();
                        dummy(qi * offset_quad + m * (n+1) + j, 2) = (1-a) * initTraj[qi][idx].z() + a * initTraj[qi][idx+1].z();
                    }
                    m++;
                }
                idx++;
            }
        }
    }

    void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, int gi) {
        IloEnv env = model.getEnv();
        for (int k = 0; k < outdim; k++) {
            for (int qi = gi * param.batch_size; qi < (gi + 1) * param.batch_size; qi++) {
                for (int m = 0; m < M; m++) {
                    for (int i = 0; i < n + 1; i++) {
                        x.add(IloNumVar(env, -IloInfinity, IloInfinity));

                        int row = k * offset_dim + (qi - gi * param.batch_size) * offset_quad + m * (n + 1) + i;
                        std::string name;
                        if (k == 0) {
                            name = "x_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                        }
                        else if (k == 1) {
                            name = "y_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                        }
                        else if (k == 2) {
                            name = "z_" + std::to_string(qi) + "_" + std::to_string(m) + "_" + std::to_string(i);
                        }
                        else{
                            ROS_ERROR("????");
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
            for (int qi = gi * param.batch_size; qi < (gi + 1) * param.batch_size; qi++) {
                for (int m = 0; m < M; m++) {
                    Eigen::MatrixXd Q_p = build_Q_p(qi, m);

                    for (int i = 0; i < n + 1; i++) {
                        int row = qi * M * (n + 1) + m * (n + 1) + i;
                        int row_idx = k * offset_dim + (qi - gi * param.batch_size) * offset_quad + m * (n + 1) + i;

                        for (int j = 0; j < n + 1; j++) {
                            int col = qi * M * (n + 1) + m * (n + 1) + j;
                            int col_idx = k * offset_dim + (qi - gi * param.batch_size) * offset_quad + m * (n + 1) + j;

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
            for (int qi = 0; qi < param.batch_size; qi++) {
                for (int i = 0; i < 2 * phi + (M - 1) * phi; i++) {
                    IloNumExpr expr(env);
                    for (int j = 0; j < M * (n + 1); j++) {
                        if (Aeq_base(i, j) != 0) {
                            expr += Aeq_base(i, j) * x[k * offset_dim + qi * offset_quad + j];
                        }
                    }
                    c.add(expr == deq((gi * param.batch_size + qi) * (2 * phi + (M - 1) * phi) + i, k));
                    expr.end();
                }
            }
        }
        count_eq = c.getSize();
//        for(int qi = 0; qi < N; qi++){
//            if(qi >= gi * plan_batch_size && qi < (gi+1) * plan_batch_size){
//                continue;
//            }
//
//            for(int j = 0; j < M*(n+1); j++){
//                c.add(x[qi*M*(n+1) + j] == dummy(qi*M*(n+1) + j, k));
//            }
//        }

        // Inequality Constraints
        for (int k = 0; k < outdim; k++) {
            for (int qi = 0; qi < N; qi++) {
                if (qi < gi * param.batch_size || qi >= (gi + 1) * param.batch_size) {
                    continue;
                }
                for (int j = 0; j < (n + 1) * M; j++) {
                    int idx = k * offset_dim + (qi - gi * param.batch_size) * offset_quad + j;
                    c.add(x[idx] <= dlq(2 * qi * offset_quad + j, k));
                    c.add(-x[idx] <= dlq((2 * qi + 1) * offset_quad + j, k));
                }
            }
        }
        int offset_box = 2 * N * offset_quad;
        int iter = 0;
        for (int qi = 0; qi < N; qi++) {
            for (int qj = qi+1; qj < N; qj++) {
                if((qi < gi * param.batch_size || qi >= (gi+1) * param.batch_size)
                   && (qj < gi * param.batch_size || qj >= (gi+1) * param.batch_size)){

                }
                else if((qi >= gi * param.batch_size && qi < (gi+1) * param.batch_size)
                        && (qj < gi * param.batch_size || qj >= (gi+1) * param.batch_size)){
                    for (int j = 0; j < M * (n + 1); j++) {
                        int idx = (qi - gi * param.batch_size) * offset_quad + j;

                        c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) * (dummy(qj * offset_quad + j, 0) - x[0 * offset_dim + idx]) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 1) * (dummy(qj * offset_quad + j, 1) - x[1 * offset_dim + idx]) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 2) * (dummy(qj * offset_quad + j, 2) - x[2 * offset_dim + idx])
                              >= mission.quad_size[qi] + mission.quad_size[qj]);
                    }
                }
                else if((qi < gi * param.batch_size || qi >= (gi+1) * param.batch_size)
                        && (qj >= gi * param.batch_size && qj < (gi+1) * param.batch_size)){
                    for (int j = 0; j < M * (n + 1); j++) {
                        int idx = (qj - gi * param.batch_size) * offset_quad + j;

                        c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) * (x[0 * offset_dim + idx] - dummy(qi * offset_quad + j, 0)) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 1) * (x[1 * offset_dim + idx] - dummy(qi * offset_quad + j, 1)) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 2) * (x[2 * offset_dim + idx] - dummy(qi * offset_quad + j, 2))
                              >= mission.quad_size[qi] + mission.quad_size[qj]);
                    }
                }
                else{
                    for (int j = 0; j < M * (n + 1); j++) {
                        int idx = (qi - gi*param.batch_size) * offset_quad + j;
                        int jdx = (qj - gi*param.batch_size) * offset_quad + j;

                        c.add(dlq(offset_box + offset_quad * 2 * iter + j, 0) * (x[0 * offset_dim + jdx] - x[0 * offset_dim + idx]) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 1) * (x[1 * offset_dim + jdx] - x[1 * offset_dim + idx]) +
                              dlq(offset_box + offset_quad * 2 * iter + j, 2) * (x[2 * offset_dim + jdx] - x[2 * offset_dim + idx])
                              >= mission.quad_size[qi] + mission.quad_size[qj]);
                    }
                }

                iter++;
            }
        }
        model.add(c);

        count_lq = c.getSize() - count_eq;
    }

    void timeMatrix(double t, Eigen::MatrixXd& tm){ //TODO: naming duplicated
        tm = Eigen::MatrixXd::Zero(n+1, n+1);

        for(int i = 0; i < n+1; i++){
            tm(i,i) = pow(t, n-i);
        }
    }

    void derivative(int qi, int k, int m, Eigen::MatrixXd& coef_der){
        coef_der = Eigen::MatrixXd::Zero(phi + 1, n + 1);
        for(int i = 0; i < phi + 1; i++){
            for(int j = 0; j < n + 1; j++){
                if(i <= j)
                    coef_der(i, n-j) = derivative_coef(i, j) * coef[qi](m * offset_seg + n-j, k);
                else
                    coef_der(i, n-j) = 0;
            }
        }
    }

    int derivative_coef(int i, int j)
    {
        return (i == 0) ? 1 : derivative_coef(i-1, j-1) * j;
    }

    std::vector<double> roots(int n_poly, const Eigen::MatrixXd& coef_der){
        std::vector<double> roots_der;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_poly, n_poly);
        for(int j = 0; j < n_poly; j++){
            if(j < n_poly-1) {
                A(j + 1, j) = 1;
            }
            if(coef_der(n_poly-1, 0) != 0) {
                A(0, j) = -coef_der(n_poly - 1, j + 1) / coef_der(n_poly - 1, 0);
            }
            else{
                ROS_ERROR("RBPPlanner: root error");
                return roots_der;
            }
        }

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        for(int i = 0; i < n_poly; i++) {
            complex<double> lambda = es.eigenvalues()[i];
            if(lambda.imag() == 0){
                roots_der.emplace_back(lambda.real());
            }
        }
    }

    double scale_to_max_vel(int qi, int k, int m, const Eigen::MatrixXd& coef_der){
        assert(phi == 3 && n == 5);
        double scale_update_rate = 1.1;

        // Get maximum accelaration
        double vel_max, t_max = 0;

        std::vector<double> ts = roots(3, coef_der);
        ts.emplace_back(0);
        ts.emplace_back(T[m+1] - T[m]);
        for(auto t : ts){
            if(t < 0 || t > T[m+1] - T[m]){
                continue;
            }

            double vel = 0;
            for(int i = 0; i < 5; i++) {
                vel += coef_der(1, i) * pow(t, 4-i);
            }
            vel = abs(vel);
            if(vel_max < vel){
                vel_max = vel;
                t_max = t;
            }
        }


        // time_scale update
        double time_scale = 1;
        while(vel_max > mission.max_vel[qi][k]){
            time_scale *= scale_update_rate;

            double vel = 0;
            for(int i = 0; i < 5; i++) {
                vel += coef_der(1, i) * pow(1/time_scale, n - i) * pow(t_max, 4-i);
            }
            vel_max = abs(vel);
        }

        return time_scale;
    }



    double scale_to_max_acc(int qi, int k, int m, const Eigen::MatrixXd& coef_der){
        assert(phi == 3 && n == 5);
        double scale_update_rate = 1.1;

        // Get maximum accelaration
        double a, b, c, D, acc_max, t_max = 0;
        a = coef_der(3, 0);
        b = coef_der(3, 1);
        c = coef_der(3, 2);
        D = b * b - 4 * a * c;
        acc_max = 0;

        std::vector<double> ts{0, T[m+1] - T[m]};
        if(D >= 0){
            ts.emplace_back((-b + sqrt(D)) / (2 * a));
            ts.emplace_back((-b - sqrt(D)) / (2 * a));
        }
        for(auto t : ts){
            if(t < 0 || t > T[m+1] - T[m]){
                continue;
            }

            double acc = 0;
            for(int i = 0; i < 4; i++) {
                acc += coef_der(2, i) * pow(t, 3-i);
            }
            acc = abs(acc);
            if(acc_max < acc){
                acc_max = acc;
                t_max = t;
            }
        }

        // time_scale update
        double time_scale = 1;
        while(acc_max > mission.max_acc[qi][k]){
            time_scale *= scale_update_rate;

            double acc = 0;
            for(int i = 0; i < 4; i++) {
                acc += coef_der(2, i) * pow(1/time_scale, n - i) * pow(t_max, 3-i);
            }
            acc_max = abs(acc);
        }

        return time_scale;
    }
};