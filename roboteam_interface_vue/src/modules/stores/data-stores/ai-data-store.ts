import {defineStore} from "pinia";
import {proto} from "../../../generated/proto";
import {computed, readonly, shallowRef} from "vue";
import IAIState = proto.IAIState;
import {Colors} from "../../field/field-objects";

export const useAIDataStore = defineStore('aiDataStore', () => {
    const state = shallowRef<IAIState | null>({});

    // Actions
    const
        updateStateFromProto = (msg: proto.IAIState) => {
            console.log('updateStateFromProto', msg);
            state.value = msg;
        },
        $reset = () => {
            state.value = null;
        }

    // Getters
    const
        fieldOrientation = computed(() => {
            return state.value?.gameSettings?.isLeft
                ? {x: 1, y: -1, angle: 0}
                : {x: -1, y: 1, angle: 180};
        });

    return {
        state: readonly(state),
        updateStateFromProto,
        $reset,
        fieldOrientation,
    }
})