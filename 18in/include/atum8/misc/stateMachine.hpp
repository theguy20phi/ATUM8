#pragma once

#include <iostream>
#include <utility>
#include <functional>
#include <unordered_map>

namespace atum8
{
    template <typename State>
    class StateMachine
    {
    public:
        using Transitions = std::unordered_map<State, std::function<State()>>;

        StateMachine(const State &startingState, const Transitions &iTransitions) : state{startingState}, transitions{iTransitions} {}

        State transition()
        {
            state = transitions[state]();
            return state;
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