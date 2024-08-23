/*
================================================================================
    File: iLQR.h
    Author: David Russell
    Date: January 23, 2024
    Description:
        My implementation of the iLQR algorithm. Heavily inspired by the paper
        "Synthesis and Stabilization of Complex Behaviors through
        Online Trajectory Optimization"
        (https://homes.cs.washington.edu/~todorov/papers/TassaIROS12.pdf).

        Some small differences, we still add lambda to Q_uu matrix diagonal instead
        of V_xx. I limit the line searches to a small set of values for speed. Lambda update
        law isn't currently quadratically updated, but perhaps it should be.

        The algorithm is as follows:
            1. Initialise the trajectory with some initial controls
            2. Rollout the trajectory and calculate the cost
            3. Compute the first order dynamics derivatives (f_x, f_u) and the
                second order cost derivatives (l_x, l_u, l_xx, l_uu).
            4. Backwards pass to calculate optimal feedback control law
            5. Forwards pass with line search to find new locally optimal trajectory
            6. Repeat steps 3-5 until convergence

        This class uses approximation of dynamics derivatives via the
        Keypoint Generator class to speed up dynamics derivative computation.
================================================================================
*/
#pragma once

#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>
#include <future>

class iLQR_SVR{
public:
    /**
     * Construct a new iLQR with state vector reduction optimiser object.
     *
     */
    iLQR_SVR(std::shared_ptr<ModelTranslator> _modelTranslator,
         std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
         std::shared_ptr<Differentiator> _differentiator,
         int horizon,
         std::shared_ptr<Visualiser> _visualizer,
         std::shared_ptr<FileHandler> _yamlReader);

    /**
     * Rollout the trajectory from an initial starting state and control sequence. Return the cost of the trajectory.
     *
     *
     * @param d - The MuJoCo data of the simulation which should be the starting state of this rollout.
     * @param save_states - Whether or not to save the states of the rollout to both X_old, and the simulator data vector.
     * @param initial_controls - The control sequence to apply from the initial state.
     *
     * @return double - The rolling cost of the trajectory.
     */
    double RolloutTrajectory(mjData *d, bool save_states, std::vector<MatrixXd> initial_controls);

    /**
     * Optimise the current trajectory until convergence, or max iterations has been reached. Uses the normal iLQR algorithm
     * to optimise the trajectory. Step 1 - Compute derivatives, Step 2 - backwards pass, Step 3 - forwards pass with linesearch.
     * Step 4 - check for convergence.
     *
     * @param initial_data_index - The data index of the simulation data which should be the starting state of optimisation.
     * @param initial_controls - The initial "warm start" trajectory to optimise from.
     * @param max_iterations - Maximum number of optimisation iterations.
     * @param min_iterations - Minimum number of optimisation iterations.
     * @param horizon_length - Horizon length to optimise to.
     *
     * @return std::vector<MatrixXd> - The new optimal control sequence.
     */
    std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length);

    void Iteration(int iteration_num, bool &converged, bool &lambda_exit);

    static void PrintBanner(double time_rollout);

    void PrintBannerIteration(int iteration, double _new_cost, double _old_cost, double eps,
                              double _lambda, int num_dofs, double time_derivs, double time_bp,
                              double time_fp, double best_alpha);

    void Resize(int new_num_dofs, int new_num_ctrl, int new_horizon);

    std::string ReturnName(){
        return "iLQR_SVR";
    }

    /**
     * Compute the new optimal control feedback law K and k from the end of the trajectory to the beginning.
     *
     * @return bool - true if successful (all matrices were P.D), false otherwise.
     */
    bool BackwardsPassQuuRegularisation();



    // ------------------------------------------------------------------------------------
    //                                 Variables
    // ------------------------------------------------------------------------------------

    double avg_surprise = 0.0;
    double avg_expected = 0.0;
    bool cost_reduced_last_iter = true;

    bool verbose_output = true;


    // ------- Timing variables --------------
    double opt_time_ms;
    std::vector<double> time_get_derivs_ms;
    double avg_time_get_derivs_ms = 0.0;
    std::vector<double> time_backwards_pass_ms;
    double avg_time_backwards_pass_ms = 0.0;
    std::vector<double> time_forwardsPass_ms;
    double avg_time_forwards_pass_ms = 0.0;

    // Saved states and controls
    vector<MatrixXd> U_old;
    vector<MatrixXd> X_new;
    vector<MatrixXd> X_old;


private:

    /**
     * Checks whether the supplied matrix is positive defeinite.
     *
     * @return bool - true if matrix is P.D, false otherwise.
     */
    bool CheckMatrixPD(Ref<MatrixXd> matrix);

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * sequentially over different alpha values to try find a new optimal sequence of controls.
     *
     * @param _old_cost - Previous cost of the old trajectory.
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPass(double _old_cost);

    /**
     * Rollout the new feedback law from the starting state of optimisation. This function performs a line search
     * in parallel over different alpha values to try find a new optimal sequence of controls.
     *
     * @param thread_id - id of the thread
     * @param alpha - Linesearch parameter between 0 and 1 ofr openloop feedback
     *
     * @return double - The cost of the new trajectory.
     */
    double ForwardsPassParallel(int thread_id, double alpha);

    /**
     * Returns a vector list of the least important DoFs in the state vector. The list is sorted.
     *
     * @return std::vector<std::string> - The least important DoFs in the state vector.
     */
    std::vector<std::string> LeastImportantDofs();


    bool UpdateLambda(bool valid_backwards_pass);

    /**
     * Computes the dynamics derivatives of the system  over the entire trajectory.
     *
     */
    void ComputeDynamicsDerivativesAtKeypoints();

    /**
     * Computes the residual derivatives over the entire trajectory.
     */
    void ComputeResidualDerivatives();

    void SaveSystemStateToRolloutData(mjData *d, int thread_id, int data_index);

    void SaveBestRollout(int thread_id);

    /**
     * This function compares the old iteration cost with the new iteration cost and computes the gradient.
     * If the cost didnt change significantly, then the optimisation has converged.
     *
     * @param old_cost - Last iterations cumulative cost.
     * @param new_cost - Current iterations cumulative cost.
     *
     * @return bool Whether the optimisation has converged or not.
     */
    bool CheckForConvergence(double old_cost, double new_cost);


    void Reset(){
        num_dofs.clear();
        time_get_derivs_ms.clear();
        time_backwards_pass_ms.clear();
        time_forwardsPass_ms.clear();
    }

    /**
     * Worker function for computing dynamics derivatives in parallel. Uses a global variable of the keypoints
     * to know which columns of the dynamics matrices need computing at which time indices.
     *
     * @param threadId - The thread id of the worker thread.
     *
     */
    void WorkerComputeDerivatives(int threadId);

    /**
     * Worker function for computing residual derivatives in parallel.
     *
     * @param threadId - The thread id of the worker thread.
     *
     */
    void WorkerComputeResidualDerivatives(int threadId);

    /**
     * This functions generates all the dynamic derivatives and cost derivatives over the entire trajectory.
     *
     */
    void GenerateDerivatives();

    void ComputeDynamicsDerivatives();
    void ComputeCostDerivatives();

    void UpdateNominal();

    void ResampleNewDofs();
    void RemoveDofs();

    void AdjustCurrentStateVector();

    // List of differentiator function callbacks, for parallelisation.
    std::vector<void (Differentiator::*)(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                                         int dataIndex, int threadId,
                                         bool central_diff, double eps)> tasks_dynamics_derivs;

    std::vector<void (Differentiator::*)(vector<MatrixXd> &r_x, vector<MatrixXd> &r_u,
                                         int dataIndex, int tid, bool central_diff, double eps)> tasks_residual_derivs;

    // Last number of linesearches performed for print banner
    int last_iter_num_linesearches = 0;
    double last_alpha = 0.0f;

    // Feedback gains matrices
    // open loop feedback gains
    vector<MatrixXd> k;
    // State dependant feedback matrices
    vector<MatrixXd> K;

    double delta_J = 0.0f;

    double expected = 0.0f;
    double surprise = 0.0f;
    std::vector<double> surprises;
    std::vector<double> expecteds;



    double epsConverge = 0.02;

    // Visualiser object
    std::shared_ptr<Visualiser> active_visualiser;

    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    std::vector<std::vector<int>> keypointsGlobal;

    std::shared_ptr<FileHandler> activeYamlReader;
    std::shared_ptr<Differentiator> activeDifferentiator;

    std::vector<std::vector<mujoco_data_min>> rollout_data;
    int num_parallel_rollouts = 6;

    // current_iteration used for parallelisation of dynamics derivatives
    std::atomic<int> current_iteration;
    int num_threads_iterations;
    std::vector<int> timeIndicesGlobal;

    double initial_cost = 0.0;
    double cost_reduction = 0.0;

    int num_iterations;

    std::vector<int> num_dofs;
    double avg_dofs = 0.0;

    // - Top level function - ensures all derivatives are calculated over an entire trajectory by some method

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
    vector<MatrixXd> A;
    vector<MatrixXd> B;

    // First and second order cost derivatives
    vector<MatrixXd> l_x;
    vector<MatrixXd> l_xx;
    vector<MatrixXd> l_u;
    vector<MatrixXd> l_uu;

    vector<MatrixXd> residuals;
    vector<vector<MatrixXd>> r_x;
    vector<vector<MatrixXd>> r_u;

    int horizon_length = 0;

    double new_cost = 0.0;
    double old_cost = 0.0;

    int sampling_k_interval = 1;
    int num_dofs_readd = 10;
    double K_matrix_threshold = 1; // maybe 0.001 or 0.0001
    // When eigen vector 0.1, 0.2, 0.5
    // WHen just summing numbers went from 1 -> 2000
    bool eigen_vector_method = false;
//    double threshold_k_eigenvectors = 0.1;

//    keypoint_method activeKeyPointMethod;

    int dof = 0;
    int num_ctrl = 0;
    int dof_used_last_optimisation = 0;

    // Lambda value which is added to the diagonal of the Q_uu matrix for regularisation purposes.
    double lambda = 0.1;
    double max_lambda = 10.0;
    double min_lambda = 0.0001;
    double lambda_factor = 10;
};

