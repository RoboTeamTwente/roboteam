import {defineStore} from 'pinia'
import {useCssVar} from "@vueuse/core";

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
    getters: {},
})