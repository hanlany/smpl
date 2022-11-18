////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022, Shohin Mukherjee
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Shohin Mukherjee

#include <smpl/search/epase.h>

#include <algorithm>
#include <cmath>

// system includes
#include <sbpl/utils/key.h>
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>

#define VERBOSE 0
#define PRINT_STATS 1

using namespace std;

namespace smpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

EPASE::EPASE(
    DiscreteSpaceInformation* space,
    Heuristic* heur)
:
    SBPLPlanner(),
    m_space(space),
    m_heur(heur),
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_open(),
    m_incons(),
    m_curr_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity())
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
}

EPASE::~EPASE()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

int EPASE::replan(
    const TimeParameters& params,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");
    initialize();

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state, 0);
        reinitSearchState(goal_state, 0);

        start_state->g = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;

        // Insert proxy edge
        auto proxy_edge_ptr = new Edge();
        proxy_edge_ptr->parent_state_ptr = start_state;
        auto edge_key = getEdgeKey(proxy_edge_ptr);
        m_edge_map.insert(make_pair(edge_key, proxy_edge_ptr));
        m_edge_open.push(proxy_edge_ptr);

    }

    if (m_goal_state_id != m_last_goal_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();




    int err;
    // while (m_satisfied_eps > m_final_eps) {
    //     if (m_curr_eps == m_satisfied_eps) {
    //         if (!m_time_params.improve) {
    //             break;
    //         }
    //         // begin a new search iteration
    //         ++m_iteration;
    //         m_curr_eps -= m_delta_eps;
    //         m_curr_eps = std::max(m_curr_eps, m_final_eps);
    //         for (SearchState* s : m_incons) {
    //             s->incons = false;
    //             m_open.push(s);
    //         }
    //         reorderOpen();
    //         m_incons.clear();
    //         SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
    //     }
    err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
    exit();
    //     if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
    //     }
    //     if (err) {
    //         break;
    //     }
    //     SMPL_DEBUG_NAMED(SLOG, "Improved solution");
    //     m_satisfied_eps = m_curr_eps;
    // }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    // if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
    //     if (m_allow_partial_solutions && !m_open.empty()) {
    //         SearchState* next_state = m_open.min();
    //         extractPath(next_state, *solution, *cost);
    //         return !SUCCESS;
    //     }
    //     return !err;
    // }

    extractPath(goal_state, *solution, *cost);
    
    if (PRINT_STATS)
    {
        cout << "*********************" << endl;
        cout << "Planning time: " << to_seconds(m_search_time) << endl;

        
        cout << endl << "---------- Main thread times -----------" << endl;
        cout << "Total edge find time in main thread: " << m_edge_find_time   << endl;
        cout << "Average edge find time: " << m_edge_find_time/m_num_edge_found << endl;
        cout << "Avg BE check time: " << m_be_check_time/m_num_be_check << " (" << m_be_check_time << "/" << m_num_be_check << ")" << endl;
        cout << "Total OPEN check time: " << m_open_check_time << endl;
        cout << "Total lock time in main thread: " << m_lock_time_main_thread << endl;
        cout << "Total wait time: " << m_wait_time << endl;
        cout << "Avg wait time: " << m_wait_time/m_wait_num << endl;
        cout << "Total re-add popped edges to OPEN time: " << m_popped_readd_time << endl;
        cout << "Total time to insert state to BE: " << m_insert_be_time << endl; 
        cout << endl << "---------------------" << endl;

        cout << endl << "---------- Expansion thread times -----------" << endl;
        cout << "Total expansions time: " << m_expansions_time << endl;
        cout << "Total lock time in expansion threads: " << m_lock_time << endl;   

        cout << endl << "--------- Data structures sizes------------" << endl;
        cout << "Number of edges found for expansion: " << m_num_edge_found << endl;
        cout << "Last open list size: " << m_edge_open_last_size << endl;
        cout << "Max open list size: " << m_edge_open_max_size << endl;
        cout << "Last BE list size: " << m_be_last_size << endl;
        cout << "Max BE list size: " << m_be_max_size << endl;
        cout << endl << "---------------------" << endl;

        cout << "Number of times open exhaust to find edge: " << m_num_open_exhaust_to_find_edge << endl;
        cout << "Avg popped edges size: " << m_num_popped_edges/m_times_popped_edges << " (" << m_num_popped_edges << " / " << m_times_popped_edges << ")" << endl;

        cout << endl << "---------------------" << endl;

        cout << "Num expand calls : " << m_num_expand_calls << endl; 
        cout << "Average expansions time: " << m_expansions_time/m_num_expand_calls << endl;

        cout << endl << "---------------------" << endl;

        cout << "Num state expansions: " << m_num_state_expansions << endl;
        cout << "Num edge evals: " << m_num_edge_evals << endl;
        cout << "State expansions/second: " << m_num_state_expansions/to_seconds(m_search_time) << endl;
        cout << "Edge evaluations/second: " << m_num_edge_evals/to_seconds(m_search_time) << endl;    

        cout << endl << "------------- Cheap expansions -------------" << endl;

        cout << "Num cheap expansions: " << m_num_cheap_expansions << endl;
        cout << "Total cheap expansions time: " << m_cheap_expansions_time << endl;
        cout << "Average cheap expansions time: " << m_cheap_expansions_time/m_num_cheap_expansions << endl;
        cout << "Average cheap GetSucc time: " << m_cheap_get_succ_time/m_num_cheap_expansions << endl;
        cout << endl << "------------- Expensive expansions -------------" << endl ;

        cout << "Num expensive expansions: " << m_num_exp_expansions << endl;
        cout << "Total expensive expansions time: " << m_exp_expansions_time << endl;
        cout << "Average expensive expansions time: " << m_exp_expansions_time/m_num_exp_expansions << endl;
        cout << "Average expensive GetSucc time: " << m_exp_get_succ_time/m_num_exp_expansions << endl;
        cout << endl << "------------- Expansions per thread -------------" << endl;
        for (int tidx = 0; tidx < m_num_threads; ++tidx)
            cout << "thread: " << tidx << " expansions: " << m_num_expansions_per_thread[tidx] << endl;
       
        cout << "*********************" << endl;
    }

    return !SUCCESS;
}

int EPASE::replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int EPASE::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    TimeParameters tparams = m_time_params;
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    return replan(tparams, solution, cost);
}

int EPASE::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int EPASE::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
    convertReplanParamsToTimeParams(params, tparams);
    return replan(tparams, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int EPASE::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open.clear();
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

/// Return the suboptimality bound of the current solution for the current search.
double EPASE::get_solution_eps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int EPASE::get_n_expands() const
{
    return m_expand_count;
}

/// Return the initial suboptimality bound
double EPASE::get_initial_eps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double EPASE::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double EPASE::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int EPASE::get_n_expands_init_solution()
{
    return m_expand_count_init;
}

/// Return the final suboptimality bound.
double EPASE::get_final_epsilon()
{
    return m_final_eps;
}

/// Return statistics for each completed search iteration.
void EPASE::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
//    stats.cost; // TODO: implement
    stats.expands = m_num_state_expansions;
    stats.time = to_seconds(m_search_time);
    stats.edge_expands = m_num_edge_evals;

    s->push_back(stats);
}

/// Set the desired suboptimality bound for the initial solution.
void EPASE::set_initialsolution_eps(double eps)
{
    m_initial_eps = eps;
}

/// Set the number of threads epase can use
void EPASE::set_num_threads(int num_threads)
{
    m_num_threads = num_threads;
}

/// Set the goal state.
int EPASE::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int EPASE::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int EPASE::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int EPASE::set_search_mode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void EPASE::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

// Recompute heuristics for all states.
void EPASE::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            s->h = m_heur->GetGoalHeuristic(s->state_id, 0);
        }
    }
}

// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void EPASE::convertTimeParamsToReplanParams(
    const TimeParameters& t,
    ReplanParams& r) const
{
    r.max_time = to_seconds(t.max_allowed_time_init);
    r.return_first_solution = !t.bounded && !t.improve;
    if (t.max_allowed_time_init == t.max_allowed_time) {
        r.repair_time = -1.0;
    } else {
        r.repair_time = to_seconds(t.max_allowed_time);
    }

    r.initial_eps = m_initial_eps;
    r.final_eps = m_final_eps;
    r.dec_eps = m_delta_eps;
}

// Convert ReplanParams to TimeParameters. Sets the current initial, final, and
// delta eps from ReplanParams.
void EPASE::convertReplanParamsToTimeParams(
    const ReplanParams& r,
    TimeParameters& t)
{
    t.type = TimeParameters::TIME;

    t.bounded = !r.return_first_solution;
    t.improve = !r.return_first_solution;

    t.max_allowed_time_init = to_duration(r.max_time);
    if (r.repair_time > 0.0) {
        t.max_allowed_time = to_duration(r.repair_time);
    } else {
        t.max_allowed_time = t.max_allowed_time_init;
    }

    m_initial_eps = r.initial_eps;
    m_final_eps = r.final_eps;
    m_delta_eps = r.dec_eps;
}

// Test whether the search has run out of time.
bool EPASE::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    case TimeParameters::USER:
        return m_time_params.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

void EPASE::initialize()
{
    m_terminate = false;
    m_recheck_flag = true;
    m_num_state_expansions = 0;
    m_num_edge_evals = 0;
    m_num_expand_calls = 0;
    m_num_cheap_expansions = 0;
    m_num_exp_expansions = 0;
    m_be_last_size = 0; 
    m_be_max_size = 0; 
    m_edge_open_last_size  = 0;
    m_edge_open_max_size  = 0;
    m_num_popped_edges = 0;
    m_times_popped_edges = 0;
    m_wait_num = 0;
    m_num_be_check = 0;
    m_reexpand = false;

    m_edge_find_time = 0.0;
    m_num_edge_found = 0;
    m_expansions_time = 0.0;
    m_cheap_get_succ_time = 0.0;
    m_cheap_expansions_time = 0.0;
    m_exp_get_succ_time = 0.0;
    m_exp_expansions_time = 0.0;
    m_lock_time = 0.0;
    m_lock_time_main_thread = 0.0;
    m_wait_time = 0.0;
    m_be_check_time = 0.0;
    m_open_check_time = 0.0;
    m_popped_readd_time = 0.0;
    m_insert_be_time = 0.0;

    m_num_open_exhaust_to_find_edge = 0.0;

    m_edge_expansion_vec.clear();
    m_edge_expansion_vec.resize(m_num_threads, NULL);
    
    m_edge_expansion_status.clear();
    m_edge_expansion_status.resize(m_num_threads, 0);
    
    m_num_expansions_per_thread.clear();
    m_num_expansions_per_thread.resize(m_num_threads, 0);

    m_edge_expansion_futures.clear();
    m_edge_expansion_futures.resize(1);
    m_being_expanded_states.clear();
    
    m_lock_vec = move(vector<LockType>(m_num_threads));
    m_cv_vec = move(vector<condition_variable>(m_num_threads));

}

void EPASE::notifyMainThread()
{
    m_recheck_flag = true;
    m_cv.notify_one();
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int EPASE::improvePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{

    vector<EdgePtrType> popped_edges;
    vector<StatePtrType> popped_be_states;

    m_lock.lock();

    while (!m_terminate) 
    {
        EdgePtrType min_edge_ptr = NULL;
        auto now = clock::now();
        double local_lock_time = 0.0;
        elapsed_time = now - start_time;

        while (!min_edge_ptr && !m_terminate)
        {

            // cout << "eopen size: " << m_edge_open.size() << endl;
            // cout << "be size: " << m_being_expanded_states.size() << endl;

            if (m_edge_open.empty() && m_being_expanded_states.empty())
            {
                m_terminate = true;
                auto now = clock::now();
                elapsed_time = now - start_time;
                cout << "Goal Not Reached" << endl;   
                m_lock.unlock();
                return false;
            }

            while(!min_edge_ptr && !m_edge_open.empty())
            {

                min_edge_ptr = m_edge_open.min();
                m_edge_open.pop();
                popped_edges.emplace_back(min_edge_ptr);


                if (!m_reexpand)
                {
                    if (min_edge_ptr->parent_state_ptr->being_expanded)
                        break;
                
                    // Independence check of curr_edge with edges in BE
                    auto t_be_check_s = clock::now();
                    for (auto& id_state : m_being_expanded_states)
                    {
                        if (id_state.second != min_edge_ptr->parent_state_ptr)
                        {
                            auto h_diff = computeHeuristic(id_state.second, min_edge_ptr->parent_state_ptr);
                            if (min_edge_ptr->parent_state_ptr->g > id_state.second->g + m_curr_eps*h_diff)
                            {
                                min_edge_ptr = NULL;
                                break;
                            }
                        }
                    }
                    auto t_be_check_e = clock::now();
                    m_be_check_time += to_seconds(t_be_check_e-t_be_check_s);
                    m_num_be_check++;

                    if (min_edge_ptr)
                    {
                        // Independence check of curr_edge with edges in OPEN that are in front of curr_edge
                        auto t_open_check_s = clock::now();
                        for (auto& popped_edge_ptr : popped_edges)
                        {
                            if (popped_edge_ptr->parent_state_ptr != min_edge_ptr->parent_state_ptr)
                            {
                                auto h_diff = computeHeuristic(popped_edge_ptr->parent_state_ptr, min_edge_ptr->parent_state_ptr);
                                if (min_edge_ptr->parent_state_ptr->g > popped_edge_ptr->parent_state_ptr->g + m_curr_eps*h_diff)
                                {
                                    // state_to_expand_found = false;
                                    min_edge_ptr = NULL;
                                    break;
                                }
                            }
                        }
                        auto t_open_check_e = clock::now();
                        m_open_check_time += to_seconds(t_open_check_e-t_open_check_s);

                    }                    
                }
            }


            // cout << "popped size: " << popped_edges.size() << endl;
            // cout << "m_edge_open size: " << m_edge_open.size() << endl;
            // cout << "min_edge_ptr: " << min_edge_ptr << endl;

            // Re add the popped states except curr state which will be expanded now
            auto t_readd_pop_s = clock::now();
            for (auto& popped_edge_ptr : popped_edges)
            {
                if (popped_edge_ptr != min_edge_ptr)
                    m_edge_open.push(popped_edge_ptr);
            }
            auto t_readd_pop_e = clock::now();
            m_popped_readd_time += to_seconds(t_readd_pop_e-t_readd_pop_s);

            m_num_popped_edges += popped_edges.size();
            m_times_popped_edges++;
            
            popped_edges.clear();

            if (!min_edge_ptr)
            {
                m_lock.unlock();

                auto t_lock_s = clock::now();                
                unique_lock<mutex> locker(m_lock);
                auto t_lock_e = clock::now();                
                local_lock_time += to_seconds(t_lock_e - t_lock_s);

                auto t_wait_s = clock::now();                
                m_cv.wait(locker, [this](){return (m_recheck_flag==true);});
                auto t_wait_e = clock::now();                
                m_wait_time += to_seconds(t_wait_e - t_wait_s);
                m_wait_num++;

                m_recheck_flag = false;
                locker.unlock();

                t_lock_s = clock::now();                
                m_lock.lock();
                t_lock_e = clock::now();                
                local_lock_time += to_seconds(t_lock_e - t_lock_s);

                continue;
            }
            

            // path to goal found
            if (min_edge_ptr->parent_state_ptr->f >= goal_state->f || min_edge_ptr->parent_state_ptr == goal_state) 
            {
                SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
                m_terminate = true;
                m_recheck_flag = true;
                auto now = clock::now();
                elapsed_time = now - start_time;
                m_lock.unlock();
                return SUCCESS;
            }

            if (timedOut(elapsed_expansions, elapsed_time)) 
            {
                SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
                m_terminate = true;
                m_recheck_flag = true;
                auto now = clock::now();
                elapsed_time = now - start_time;
                m_lock.unlock();
                return TIMED_OUT;
            }

            if (m_edge_open.empty() && !min_edge_ptr)
                m_num_open_exhaust_to_find_edge++;

        }


        // Insert the state in BE and mark it closed if the edge being expanded is dummy edge
        auto t_add_be_s = clock::now();
        if (min_edge_ptr->action_idx == -1)
        {
            min_edge_ptr->parent_state_ptr->is_visited += 1;
            min_edge_ptr->parent_state_ptr->being_expanded = true;
            m_being_expanded_states.insert(make_pair(min_edge_ptr->parent_state_ptr->state_id, min_edge_ptr->parent_state_ptr));
        }
        auto t_add_be_e = clock::now();
        m_insert_be_time += to_seconds(t_add_be_e-t_add_be_s);


        auto now_edge_found = clock::now(); 
        m_edge_find_time += to_seconds(now_edge_found - now) - local_lock_time;
        m_lock_time_main_thread += local_lock_time;
        m_num_edge_found++;
        m_edge_open_max_size = max(m_edge_open_max_size, (int)m_edge_open.size());
        m_be_max_size = max(m_be_max_size, (int)m_being_expanded_states.size());
        
        m_lock.unlock();


        if (VERBOSE) cout << "Num state expansions: " << m_num_state_expansions << endl;

        // if ((m_num_threads == 1) || (min_edge_ptr->action_idx == -1))
        if (m_num_threads == 1)
        {
            expandEdge(min_edge_ptr, 0);
        }
        else
        {
            int thread_id = 1;
            bool edge_expansion_assigned = false;
            while (!edge_expansion_assigned)
            {
                unique_lock<mutex> locker(m_lock_vec[thread_id]);
                auto status = m_edge_expansion_status[thread_id];
                locker.unlock();                


                if (!status)
                {
                    if (thread_id >= m_edge_expansion_futures.size())
                    {
                        if (1) cout << "Spawning edge expansion thread " << thread_id << endl;
                        m_edge_expansion_futures.emplace_back(async(launch::async, &EPASE::expandEdgeLoop, this, thread_id));
                    }
                    locker.lock();
                    m_edge_expansion_vec[thread_id] = min_edge_ptr;
                    m_edge_expansion_status[thread_id] = 1;
                    edge_expansion_assigned = true;       
                    locker.unlock();
                    m_cv_vec[thread_id].notify_one();
                }
                else
                    thread_id = thread_id == m_num_threads-1 ? 1 : thread_id+1;

            }
        }

        auto t_lock_s = clock::now();
        m_lock.lock();
        auto t_lock_e = clock::now();
        elapsed_expansions = m_num_state_expansions; 
    }

    m_terminate = true;
    auto now = clock::now();
    elapsed_time = now - start_time;
    m_lock.unlock();
    return EXHAUSTED_OPEN_LIST;
}

void EPASE::expandEdgeLoop(int thread_id)
{
    while (!m_terminate)
    {

        unique_lock<mutex> locker(m_lock_vec[thread_id]);
        m_cv_vec[thread_id].wait(locker, [this, thread_id](){return (m_edge_expansion_status[thread_id] == 1);});
        locker.unlock();

        // while ((!status) && (!m_terminate))
        // {
        //     m_lock_vec[thread_id].lock();
        //     status = m_edge_expansion_status[thread_id];
        //     m_lock_vec[thread_id].unlock();
        //     // cout << "Expansion thread " << thread_id << " waiting! " << m_edge_expansion_status[thread_id] << endl;
        // }

        if (m_terminate)
            break;


        expandEdge(m_edge_expansion_vec[thread_id], thread_id);

        locker.lock();
        m_edge_expansion_vec[thread_id] = NULL;
        m_edge_expansion_status[thread_id] = 0;
        locker.unlock();

    }    
}


void EPASE::reexpandEdge(EdgePtrType edge_ptr)
{
    if (VERBOSE) edge_ptr->Print("Re-expanding edge", true);

    int new_g = edge_ptr->parent_state_ptr->g + edge_ptr->cost;
    if (new_g < edge_ptr->child_state_ptr->g)
    {
        edge_ptr->child_state_ptr->g= new_g;
        edge_ptr->child_state_ptr->bp = edge_ptr->parent_state_ptr;
        edge_ptr->child_state_ptr->f = computeKey(edge_ptr->child_state_ptr);

        // Insert proxy edge from successor
        auto proxy_edge_ptr = new Edge();
        proxy_edge_ptr->parent_state_ptr = edge_ptr->child_state_ptr;
        auto proxy_edge_key = getEdgeKey(proxy_edge_ptr);
        auto it_edge = m_edge_map.find(proxy_edge_key);

        if (it_edge == m_edge_map.end())
        {
            throw runtime_error("Something is wrong, proxy edge from successor should already exist for reexpanded edge");
        }

        proxy_edge_ptr = it_edge->second;
        proxy_edge_ptr->exp_priority = edge_ptr->child_state_ptr->f;
        
        if (m_edge_open.contains(proxy_edge_ptr))
        {
            if (VERBOSE) proxy_edge_ptr->Print("Proxy edge already in eopen ");
            m_edge_open.decrease(proxy_edge_ptr);
        }
        else
        {
            
            if (VERBOSE) proxy_edge_ptr->Print("Inserting proxy edge into eopen ");
            m_edge_open.push(proxy_edge_ptr);
        }
        
        notifyMainThread();

    }

}

void EPASE::expandExpensiveEdge(EdgePtrType edge_ptr, int thread_id)
{
    if (VERBOSE) edge_ptr->Print("Expensive expansion", true);


    auto action_idx = edge_ptr->action_idx;

    vector<int> succ_state_id, cost;

    m_lock.unlock();
    // Evaluate the edge
    auto t_succ_s = clock::now();
    m_space->GetSucc(edge_ptr->parent_state_ptr->state_id, action_idx, &succ_state_id, &cost, thread_id);
    auto t_succ_e = clock::now();
    m_exp_get_succ_time += to_seconds(t_succ_e - t_succ_s);

    //********************
    auto t_lock_s = clock::now();
    m_lock.lock();
    auto t_lock_e = clock::now();
    m_lock_time += to_seconds(t_lock_e - t_lock_s);

    // planner_stats_.num_evaluated_edges_++; // Only the edges controllers that satisfied pre-conditions and args are in the open list

    if (!succ_state_id.empty())
    {
        SearchState* succ_state = getSearchState(succ_state_id[0]);
        reinitSearchState(succ_state, thread_id);

        edge_ptr->child_state_ptr = succ_state;
        edge_ptr->cost = cost[0];
        
        if (m_reexpand || (!succ_state->is_visited))
        {
            int new_cost = edge_ptr->parent_state_ptr->g + cost[0];
            if (new_cost < succ_state->g) 
            {
            
                succ_state->g = new_cost;
                succ_state->bp = edge_ptr->parent_state_ptr;
                succ_state->f = computeKey(succ_state);

                // Insert proxy edge from successor
                auto proxy_edge_ptr = new Edge();
                proxy_edge_ptr->parent_state_ptr = succ_state;
                auto edge_key = getEdgeKey(proxy_edge_ptr);
                auto it_edge = m_edge_map.find(edge_key); 


                if (it_edge == m_edge_map.end())
                {
                    if (VERBOSE) proxy_edge_ptr->Print("New edge ");
                    proxy_edge_ptr->edge_id = edge_key;
                    m_edge_map.insert(make_pair(edge_key, proxy_edge_ptr));
                }
                else
                {
                    delete proxy_edge_ptr;
                    proxy_edge_ptr = it_edge->second;
                }

                proxy_edge_ptr->exp_priority = succ_state->f;
                
                if (m_edge_open.contains(proxy_edge_ptr))
                {
                    if (VERBOSE) proxy_edge_ptr->Print("Proxy edge already in eopen ");
                    m_edge_open.decrease(proxy_edge_ptr);
                }
                else
                {
                    
                    if (VERBOSE) proxy_edge_ptr->Print("Inserting proxy edge into eopen ");
            
                    m_edge_open.push(proxy_edge_ptr);
                }

                notifyMainThread();
            }
        }
        

    }
  
}

void EPASE::expandCheapEdges(EdgePtrType& edge_ptr, vector<int>& action_idx_vec, int thread_id)
{
    if (VERBOSE) edge_ptr->Print("Cheap expansion", true);

    vector<int> costs;
    vector<int> succs;

    m_lock.unlock();
    auto t_succ_s = clock::now();
    m_space->GetSuccs(edge_ptr->parent_state_ptr->state_id, action_idx_vec, &succs, &costs, thread_id);
    auto t_succ_e = clock::now();
    m_cheap_get_succ_time += to_seconds(t_succ_e - t_succ_s);
 
    auto t_lock_s = clock::now();
    m_lock.lock();
    auto t_lock_e = clock::now();
    m_lock_time += to_seconds(t_lock_e - t_lock_s);

    for (auto succ_idx = 0; succ_idx < succs.size(); ++succ_idx)
    {
        SearchState* succ_state = getSearchState(succs[succ_idx]);
        reinitSearchState(succ_state, thread_id);

        auto edge_ptr_real = new Edge();
        edge_ptr_real->action_idx = action_idx_vec[succ_idx];
        edge_ptr_real->parent_state_ptr = edge_ptr->parent_state_ptr;
        edge_ptr_real->exp_priority = edge_ptr->exp_priority;
        edge_ptr_real->child_state_ptr = succ_state;
        edge_ptr_real->cost = costs[succ_idx];  
        edge_ptr_real->edge_id = getEdgeKey(edge_ptr_real);
        m_edge_map.insert(make_pair(edge_ptr_real->edge_id, edge_ptr_real));



        // // test
        // edge_ptr_real->Print("Edge should be in map");
        // (*m_edge_map.find(edge_ptr_real->edge_id)).second->Print("This edge should now have child_state_ptr");
        // getchar();



        
        if (m_reexpand || (!succ_state->is_visited))
        {
            int new_cost = edge_ptr->parent_state_ptr->g + costs[succ_idx];
            if (new_cost < succ_state->g) 
            {
         
                succ_state->g = new_cost;
                succ_state->bp = edge_ptr->parent_state_ptr;
                succ_state->f = computeKey(succ_state);

                // Insert proxy edge
                auto proxy_edge_ptr = new Edge();
                proxy_edge_ptr->parent_state_ptr = succ_state;
                auto edge_key = getEdgeKey(proxy_edge_ptr);
                auto it_edge = m_edge_map.find(edge_key); 


                if (it_edge == m_edge_map.end())
                {
                    if (VERBOSE) proxy_edge_ptr->Print("New edge ", true);
                    proxy_edge_ptr->edge_id = edge_key;
                    m_edge_map.insert(make_pair(edge_key, proxy_edge_ptr));
                }
                else
                {
                    delete proxy_edge_ptr;
                    proxy_edge_ptr = it_edge->second;
                }

                proxy_edge_ptr->exp_priority = succ_state->f;
                
                if (m_edge_open.contains(proxy_edge_ptr))
                {
                    if (VERBOSE) proxy_edge_ptr->Print("Proxy edge already in eopen ", true);
                    m_edge_open.decrease(proxy_edge_ptr);
                }
                else
                {
                    
                    if (VERBOSE) proxy_edge_ptr->Print("Inserting proxy edge into eopen ", true);
            
                    m_edge_open.push(proxy_edge_ptr);
                }
                
                notifyMainThread();

            }
        }

    }

}


void EPASE::expandEdge(EdgePtrType& edge_ptr, int thread_id)
{
    auto t_start = clock::now(); 

    auto t_lock_s = clock::now();
    m_lock.lock();
    auto t_lock_e = clock::now();
    m_lock_time += to_seconds(t_lock_e - t_lock_s);

    m_num_expand_calls += 1;
    m_num_expansions_per_thread[thread_id] += 1;
    
    if (VERBOSE) 
    {
        cout << "------------------" << endl;
        edge_ptr->Print("Expanding", true);
        cout << "------------------" << endl;
    }
    
    // Proxy edge, add the real edges to Eopen
    if (edge_ptr->action_idx == -1)
    {       
        if (VERBOSE) edge_ptr->Print("Proxy expansion");
        m_num_state_expansions += 1;

        // int num_succs;
        // m_space->GetNumSuccs(edge_ptr->parent_state_ptr->state_id, num_succs);
        vector<int> cheap_succs;
        vector<int> expensive_succs;
        m_space->GetCheapExpensiveSuccsIdxs(edge_ptr->parent_state_ptr->state_id, cheap_succs, expensive_succs);
        edge_ptr->parent_state_ptr->num_successors = cheap_succs.size() + expensive_succs.size();


        // if (edge_ptr->parent_state_ptr->is_visited > 1)
        // {
        //     edge_ptr->Print("Re-expanding state", true);
        // }

        // Re-expand cheap successors
        for (auto sit = cheap_succs.begin(); sit < cheap_succs.end(); ++sit)
        {
            auto edge_ptr_real = new Edge();
            edge_ptr_real->action_idx = *sit;
            edge_ptr_real->parent_state_ptr = edge_ptr->parent_state_ptr;
            edge_ptr_real->exp_priority = edge_ptr->exp_priority;
            edge_ptr_real->edge_id = getEdgeKey(edge_ptr_real);
            auto it_edge = m_edge_map.find(edge_ptr_real->edge_id);


            if (it_edge != m_edge_map.end())
            {
                delete edge_ptr_real;
                edge_ptr_real = it_edge->second;

                if (edge_ptr_real->child_state_ptr)
                {
                    // cout << " HERE" << endl;
                    reexpandEdge(edge_ptr_real);
                    sit = cheap_succs.erase(sit);
                }
                
            }
            else
            {
                delete edge_ptr_real;
            }


        }


        // Add expensive edges to open
        for (auto sidx: expensive_succs) 
        {
            auto edge_ptr_real = new Edge();
            edge_ptr_real->action_idx = sidx;
            edge_ptr_real->parent_state_ptr = edge_ptr->parent_state_ptr;
            edge_ptr_real->exp_priority = edge_ptr->exp_priority;
            edge_ptr_real->edge_id = getEdgeKey(edge_ptr_real);

            auto it_edge = m_edge_map.find(edge_ptr_real->edge_id);
            if (it_edge == m_edge_map.end())
            {
                m_edge_map.insert(make_pair(edge_ptr_real->edge_id, edge_ptr_real));
                m_edge_open.push(edge_ptr_real);
                notifyMainThread();
            }
            else
            {
                delete edge_ptr_real;
                edge_ptr_real = it_edge->second;
                
                if (edge_ptr_real->child_state_ptr)
                {
                    reexpandEdge(edge_ptr_real);
                    cheap_succs.erase(cheap_succs.begin()+sidx);
                }
                else
                {
                    if (VERBOSE) edge_ptr_real->Print("Inserting real edge into eopen ");
                    
                    edge_ptr_real->exp_priority = edge_ptr->exp_priority;

                    if (m_edge_open.contains(edge_ptr_real))
                        m_edge_open.decrease(edge_ptr_real);                    
                    else
                        m_edge_open.push(edge_ptr_real);                    
            
                    notifyMainThread();
                }
                

            }
        }

        // Expand cheap successors
        auto t_start = clock::now(); 
        expandCheapEdges(edge_ptr, cheap_succs, thread_id);
        auto t_end = clock::now(); 
        m_cheap_expansions_time += to_seconds(t_end - t_start);
        m_num_cheap_expansions+=1;

        edge_ptr->parent_state_ptr->num_expanded_successors += cheap_succs.size();
        m_num_edge_evals+=cheap_succs.size();


        // cout << "eopen size: " << m_edge_open.size();

        // num_proxy_expansions_++; 
    }
    else
    { 
        // Real edge, evaluate and add proxy edges for child 


        auto t_start = clock::now(); 
        expandExpensiveEdge(edge_ptr, thread_id);
        auto t_end = clock::now(); 
        m_exp_expansions_time += to_seconds(t_end - t_start);
        m_num_exp_expansions+=1;

        edge_ptr->parent_state_ptr->num_expanded_successors += 1;
        m_num_edge_evals+=1;

    }
  

    // edge_ptr->parent_state_ptr->Print("Finished state ");

    if (!m_reexpand)
    {    
        if (edge_ptr->parent_state_ptr->num_expanded_successors == edge_ptr->parent_state_ptr->num_successors)
        {
            edge_ptr->parent_state_ptr->being_expanded = false;
            auto it_state_be = m_being_expanded_states.find(edge_ptr->parent_state_ptr->state_id);
            if (it_state_be != m_being_expanded_states.end())
            {
                m_being_expanded_states.erase(it_state_be);
                notifyMainThread();
            }
        }

        if (edge_ptr->parent_state_ptr->num_expanded_successors > edge_ptr->parent_state_ptr->num_successors)
        {
            edge_ptr->parent_state_ptr->Print();
            throw runtime_error("Number of expanded edges cannot be greater than number of successors");
        }
    }


    auto t_end = clock::now(); 
    m_expansions_time += to_seconds(t_end - t_start);

    m_lock.unlock();
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void EPASE::reorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        (*it)->f = computeKey(*it);
    }
    m_open.make();
}

int EPASE::computeKey(SearchState* s) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h);
}

size_t EPASE::getEdgeKey(const EdgePtrType& edge_ptr)
{
    size_t seed = 0;
    boost::hash_combine(seed, edge_ptr->parent_state_ptr->state_id);
    if (edge_ptr->action_idx != -1)
        boost::hash_combine(seed, edge_ptr->action_idx);
    return seed;
}

int EPASE::computeHeuristic(const StatePtrType& state_ptr)
{
    return state_ptr->h;
}

int EPASE::computeHeuristic(const StatePtrType& state_ptr_1, const StatePtrType& state_ptr_2)
{
    return abs(computeHeuristic(state_ptr_1)-computeHeuristic(state_ptr_2));
}


// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
EPASE::SearchState* EPASE::getSearchState(int state_id)
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }

    auto& state = m_states[state_id];
    if (state == NULL) {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
EPASE::SearchState* EPASE::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;

    return ss;
}

// Lazily (re)initialize a search state.
void EPASE::reinitSearchState(SearchState* state, int tidx)
{
    if (state->call_number != m_call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id, tidx);
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

// Extract the path from the start state up to a new state.
void EPASE::extractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (SearchState* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

void EPASE::exit()
{

    for (int thread_id = 1; thread_id < m_num_threads; ++thread_id)
    {
        unique_lock<mutex> locker(m_lock_vec[thread_id]);
        m_edge_expansion_status[thread_id] = 1;
        locker.unlock();
        m_cv_vec[thread_id].notify_one();
    }


    bool all_expansion_threads_terminated = false;
    while (!all_expansion_threads_terminated)
    {
        all_expansion_threads_terminated = true;
        for (int fut_idx = 1; fut_idx < m_edge_expansion_futures.size(); ++fut_idx)
        {
            if (!isFutureReady(m_edge_expansion_futures[fut_idx]))
            {
                all_expansion_threads_terminated = false;
                break;
            }
            else
                if (VERBOSE) cout << "thread exited: " << fut_idx << endl;
        }
    }
    m_edge_expansion_futures.clear();

    m_edge_open_last_size = m_edge_open.size();
    m_be_last_size = m_being_expanded_states.size();

    while (!m_edge_open.empty())
        m_edge_open.pop();
    
    cout << "Epase exited" << endl;
}


} // namespace smpl
