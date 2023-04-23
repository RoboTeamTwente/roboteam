import {defineStore} from 'pinia'
import {Colors} from "../field/field-objects";

export type GameSettingsStore = {
    useReferee: boolean,
    ignoreInvariants: boolean,
    hubMode: 'basestation' | 'simulator',
    side: 'left' | 'right',
    team: 'blue' | 'yellow',
}

export const useGameSettingsStore = defineStore('gameSettingsStore', {
    persist: true,
    state: (): GameSettingsStore => {return {
        useReferee: true,
        ignoreInvariants: false,
        hubMode: 'basestation',
        side: 'right',
        team: 'blue',
    }},
    actions: {},
    getters: {
        goalColor(state){
            return state.team === "yellow"
                ? {leftGoal: Colors.yellow, rightGoal: Colors.blue}
                : {leftGoal: Colors.blue, rightGoal: Colors.yellow};
        },
        fieldOrientation(state) {
            return state.side === "left"
                ? {x: 1, y: -1, angle: 0}
                : {x: -1, y: 1, angle: 180};
        }
    },
})