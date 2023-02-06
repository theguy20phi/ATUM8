/**
 * @file stateMachine.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides the template class for all state machines.
 * @version 0.1
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <iostream>
#include <utility>
#include <functional>
#include <unordered_map>

namespace atum8
{
    /**
     * @brief Provides the basic underlying logic for state machines.
     * Can be used for any subsystem that can be modeled by a finite state
     * machine.
     *
     * @tparam State A provided enum that represents the valid states of the machine.
     */
    template <typename State>
    class StateMachine
    {
    public:
        using Transitions = std::unordered_map<State, std::function<State()>>;

        /**
         * @brief Construct a new State Machine object
         *
         * @param startingState The initial state of the machine.
         * @param iTransitions The "automatic" transitions, given as a hashmap of states
         * and functions returnings states.
         */
        StateMachine(State startingState, Transitions iTransitions) : state{startingState}, transitions{iTransitions} {}

        /**
         * @brief Performs the transition associated with the current state.
         *
         */
        State transition()
        {
            state = transitions[state]();
            return state;
        }

        /**
         * @brief Sets the state to a new value.
         *
         * @param iState The new state of the machine.
         */
        void setState(State iState)
        {
            state = iState;
        }

        /**
         * @brief Gets the current state.
         *
         * @return state
         */
        State getState() const
        {
            return state;
        }

    private:
        State state;
        Transitions transitions;
    };
}