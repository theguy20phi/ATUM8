/**
 * @file StateMachine.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides the template class for all state machines.
 * @version 0.1
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <utility>
#include <functional>
#include <unordered_map>

namespace ATUM8
{
    template <typename State>
    class StateMachine
    {
    public:
        using Transitions = std::unordered_map<State, std::function<State()>>;

        StateMachine(State startingState, Transitions iTransitions = {}) : state(startingState), transitions(iTransitions) {}

        void transition()
        {
            state = transitions[state]();
        }

        void setState(State iState)
        {
            state = iState;
        }

        State getState() const
        {
            return state;
        }

    private:
        State state;
        Transitions transitions;
    };
}