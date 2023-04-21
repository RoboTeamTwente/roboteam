import {defineStore} from "pinia";
import { proto } from "../../generated/proto";

type WorldStoreState = {
    latest: proto.IState | null
};

export const useWorldStateStore = defineStore('stateStore', {
    state: (): WorldStoreState => {return {latest: null, latestField: null}},
    actions: {
        pushNewState(state: proto.IState) {
            this.latest = state;
        }
    }
});