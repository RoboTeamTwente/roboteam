import {defineStore} from 'pinia'
import {useCssVar} from "@vueuse/core";

export const playHistoryLimit = 10;
export const haltPlayName = 'Halt';

export type AiStore = {
    running: boolean
    play: string,
    ruleset: string,
    availablePlays: string[],
    availableRulesets: string[],

    playHistory: string[],
}
export const useAIStore = defineStore('aiStore', {
    state: (): AiStore => {return {
        running: true,
        play: 'Halt',
        ruleset: 'default',
        availablePlays: [],
        availableRulesets: [],
        playHistory: [],
    }},
    actions: {
        setRunning(running: boolean) {this.running = running},
        resetPlay() {console.log('reset play')},
    },
    getters: {}
})