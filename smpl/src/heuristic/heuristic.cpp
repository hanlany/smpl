////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

#include <smpl/heuristic/heuristic.h>

namespace smpl {

Heuristic::~Heuristic()
{
}

bool Heuristic::Init(DiscreteSpace* space)
{
    if (space == NULL) {
        return false;
    }

    m_space = space;
    return true;
}

auto Heuristic::GetPlanningSpace() -> DiscreteSpace*
{
    return m_space;
}

auto Heuristic::GetPlanningSpace() const -> const DiscreteSpace*
{
    return m_space;
}

bool Heuristic::UpdateStart(int state_id)
{
    return true;
}

bool Heuristic::UpdateGoal(GoalConstraint* goal)
{
    return true;
}

IGoalHeuristic::~IGoalHeuristic() { }

IStartHeuristic::~IStartHeuristic() { }

IPairwiseHeuristic::~IPairwiseHeuristic() { }

IMetricGoalHeuristic::~IMetricGoalHeuristic() { }

IMetricStartHeuristic::~IMetricStartHeuristic() { }

} // namespace smpl