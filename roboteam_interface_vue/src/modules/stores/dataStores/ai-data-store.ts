import {defineStore} from "pinia";
import {proto} from "../../../generated/proto";
import ISTPStatus = proto.ISTPStatus;
import {DeepReadonly} from "../../../utils";
import {computed, readonly, ref, shallowRef} from "vue";
import IAIState = proto.IAIState;
import {Colors} from "../../field/field-objects";
import {emitter} from "../../../services/ai-events";

export const useAIDataStore = defineStore('aiDataStore', () => {
    const state = shallowRef<IAIState | null>({});

    // Actions
    const updateStateFromProto = (msg: proto.IAIState) => {
        console.log('updateStateFromProto', msg);
        state.value = msg;
    }

    const $reset = () => {
        state.value = null;
    }

    const goalColor = computed(() => {
        return state.value?.gameSettings?.isYellow
            ? {leftGoal: Colors.yellow, rightGoal: Colors.blue}
            : {leftGoal: Colors.blue, rightGoal: Colors.yellow};
    });

    const fieldOrientation = computed(() => {
        return state.value?.gameSettings?.isLeft
            ? {x: 1, y: -1, angle: 0}
            : {x: -1, y: 1, angle: 180};
    });

    const useReferee = computed({
        get() { return state.value?.runtimeConfig?.useReferee!},
        set(value: boolean) {
            emitter.emit('update:runtimeConfiguration', proto.RuntimeConfig.create({
                ...state.value?.runtimeConfig,
                useReferee: value
            }));
        },
    });

    const ignoreInvariants = computed({
        get() { return state.value?.runtimeConfig?.ignoreInvariants!},
        set(value: boolean) {
            emitter.emit('update:runtimeConfiguration', proto.RuntimeConfig.create({
                ...state.value?.runtimeConfig,
                ignoreInvariants: value
            }));
        },
    });

    const isLeft = computed({
        get() { return state.value?.gameSettings?.isLeft!},
        set(value: boolean) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...state.value?.gameSettings,
                isLeft: value
            }));
        },
    });

    const isYellow = computed({
        get() { return state.value?.gameSettings?.isYellow!},
        set(value: boolean) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...state.value?.gameSettings,
                isYellow: value
            }));
        },
    });

    const robotHubMode = computed({
        get() { return state.value?.gameSettings?.robotHubMode!},
        set(value: proto.GameSettings.RobotHubMode) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...state.value?.gameSettings,
                robotHubMode: value
            }));
        },
    });

    const isPaused = computed({
        get() { return state.value?.isPaused!},
        set(value: boolean) {
            emitter.emit('update:pause', value);
        },
    });

    return {
        state: readonly(state),
        updateStateFromProto,
        $reset,
        goalColor,
        fieldOrientation,
        useReferee,
        ignoreInvariants,
        isLeft,
        isYellow,
        robotHubMode,
        isPaused
    }
})