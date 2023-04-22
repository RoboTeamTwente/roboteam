import {defineStore} from "pinia";
import { proto } from "../../generated/proto";

type WorldStoreState = {
    latest: proto.IState | null
};

export const useWorldStateStore = defineStore('stateStore', {
    state: (): WorldStoreState => {return {latest: null}},
    actions: {
        pushNewState(state: proto.IState) {
            this.latest = state;
        }
    },
    getters: {
        getLatestState(state) {
            return () => {
                if (state.latest?.lastSeenWorld === null || state.latest?.lastSeenWorld === undefined) {
                    return null;
                }

                return {
                    world: state.latest!.lastSeenWorld!,
                };
            }
        }
    }
});