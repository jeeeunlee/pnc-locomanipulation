// #include <Eigen/LU>
// #include <Eigen/SVD>

#include <my_wbc/QuadProgSolver.hpp>

QuadProgSolver::QuadProgSolver() { 
    b_initialized = false;
}

// same setting to Matlab "x = quadprog(H,f,A,b,Aeq,beq)"
// https://www.mathworks.com/help/optim/ug/quadprog.html 
// min 1/2*x'*H*x + f'*x
// s.t.
//      Ax <= b
//      Aeq*x = beq
void QuadProgSolver::setProblem(const Eigen::MatrixXd& _H,
                                const Eigen::VectorXd& _f,
                                const Eigen::MatrixXd& _Aieq,
                                const Eigen::VectorXd& _bieq,
                                const Eigen::MatrixXd& _Aeq,
                                const Eigen::VectorXd& _beq){
    n = _f.size();
    m = _bieq.size();
    p = _beq.size();

    // min 0.5 * x G x + g0 x
    // s.t.
    //     CE^T x + ce0 = 0
    //     CI^T x + ci0 >= 0
    x.resize(n);   

    // Set Cost
    G.resize(n, n); // n * n
    g0.resize(n);
    for (int i(0); i < n; ++i) {
        for (int j(0); j < n; ++j) {
            G[i][j] = _H(i, j);
        }
        g0[i] = _f[i];
    }
    // Set Constraints
    CE.resize(n, p); // n * p
    ce0.resize(p);
    for (int i(0); i < p; ++i) {
        for (int j(0); j < n; ++j) {
            CE[j][i] = _Aeq(i,j);
        }
        ce0[i] = -_beq[i];
    }
    CI.resize(n, m); // n * m
    ci0.resize(m);
    for (int i(0); i < m; ++i) {
        for (int j(0); j < n; ++j) {
            CI[j][i] = - _Aieq(i,j);
        }
        ci0[i] = _bieq[i];
    }

    b_initialized = true;
}

void QuadProgSolver::setProblem(const Eigen::MatrixXd& _H,
                    const Eigen::VectorXd& _f,
                    const Eigen::MatrixXd& _Aieq,
                    const Eigen::VectorXd& _bieq){
    n = _f.size();
    m = _bieq.size();
    p = 0;

    // min 0.5 * x G x + g0 x
    // s.t.
    //     CE^T x + ce0 = 0
    //     CI^T x + ci0 >= 0
    x.resize(n);   

    // Set Cost
    G.resize(n, n); // n * n
    g0.resize(n);
    for (int i(0); i < n; ++i) {
        for (int j(0); j < n; ++j) {
            G[i][j] = _H(i, j);
        }
        g0[i] = _f[i];
    }
    // Set Constraints
    CE.resize(n, 0); // n * p
    ce0.resize(0);
    CI.resize(n, m); // n * m
    ci0.resize(m);
    for (int i(0); i < m; ++i) {
        for (int j(0); j < n; ++j) {
            CI[j][i] = - _Aieq(i,j);
        }
        ci0[i] = _bieq[i];
    }
    b_initialized = true;
}

void QuadProgSolver::solveProblem(Eigen::VectorXd& _x){
    if(!b_initialized){
        std::cout << " setProblem required " << std::endl;
        return;
    }
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    std::cout<<" solve_quadprog done"<< std::endl;
    _x = Eigen::VectorXd::Zero(n);
    if(f == std::numeric_limits<double>::infinity())  {
        std::cout << "Infeasible Solution f: " << f << std::endl;
        std::cout << "x: " << x << std::endl;
        // exit(0.0);        
    }
    else{
        for(int i(0); i<n; ++i){
            _x[i] = x[i];
        }
    }
}