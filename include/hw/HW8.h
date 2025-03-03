#pragma once

#include "tools/Algorithms.h"
#include "tools/Environment.h"
#include "tools/Path.h"
#include "tools/Graph.h"

#include <tuple>

namespace amp {

class CentralizedMultiAgentRRT : public amp::MultiAgentCircleMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        //virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) = 0;

        virtual ~CentralizedMultiAgentRRT() {}
};

class DecentralizedMultiAgentRRT : public amp::MultiAgentCircleMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        //virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) = 0;

        virtual ~DecentralizedMultiAgentRRT() {}
};

class HW8 {
    public:
        /// @brief Get the multi-agent problem described in the assignment
        /// @param n_agents Number of agents (must be between 1 and 6) to generate with
        /// @return Multi-agent problem with the specified number of agents
        static amp::MultiAgentProblem2D getWorkspace1(uint32_t n_agents);

        /// @brief Checks the path generated by your motion planner against the problem
        /// @param path Multi-agent path solution generated by your motion planner
        /// @param prob Multi-agent problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool check(const amp::MultiAgentPath2D& ma_path, const amp::MultiAgentProblem2D& prob, bool verbose = true);

        /// @brief Checks the path generated by your motion planner against the problem
        /// @param path Multi-agent path solution generated by your motion planner
        /// @param prob Multi-agent problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @param collision_states Gather collision states found along each path
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool check(const amp::MultiAgentPath2D& ma_path, const amp::MultiAgentProblem2D& prob, std::vector<std::vector<Eigen::Vector2d>>& collision_states, bool verbose = true);

        /// @brief Generates a random mult-agent problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your multi-agent algorithm
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::MultiAgentCircleMotionPlanner2D& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your multi-agent algorithm
        /// @param ma_path Return the multi-agent path generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::MultiAgentCircleMotionPlanner2D& algo, amp::MultiAgentPath2D& ma_path, amp::MultiAgentProblem2D& prob, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your multi-agent algorithm
        /// @param ma_path Return the multi-agent path generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param collision_states Gather collision states found along each path
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::MultiAgentCircleMotionPlanner2D& algo, amp::MultiAgentPath2D& ma_path, amp::MultiAgentProblem2D& prob, std::vector<std::vector<Eigen::Vector2d>>& collision_states, bool verbose = true, uint32_t seed = 0u);

        static int grade(CentralizedMultiAgentRRT& c_ma_algo, DecentralizedMultiAgentRRT& dc_ma_algo, const std::string& email, int argc, char** argv);

        template <class C_MA_ALGO, class DC_MA_ALGO>
        static int grade(const std::string& email, int argc, char** argv);

        template <class C_MA_ALGO, class DC_MA_ALGO, class _C_MA_CTOR_ARGS_TUP, class _DC_MA_CTOR_ARGS_TUP>
        static int grade(const std::string& email, int argc, char** argv, 
                    const _C_MA_CTOR_ARGS_TUP& c_ma_ctor_args_tuple,
                    const _DC_MA_CTOR_ARGS_TUP& dc_ma_ctor_args_tuple);
    private:
        static void assertDerivesSamplerAlgo(amp::MultiAgentCircleMotionPlanner2D& algo);
};

#define AMP_HW8_ALIAS "hw8"
#define AMP_HW8_PACKAGE_NAME "hw8_report_card"
}

#include "public/HW8_impl.h"