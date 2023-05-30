import {computed} from "vue";
import {emitter} from "../../services/ai-events";
import {proto} from "../../generated/proto";
import {useAIDataStore} from "../stores/data-stores/ai-data-store";

export const useAiController = () => {
    const aiData = useAIDataStore();

    const useReferee = computed({
        get() {
            return aiData.state?.runtimeConfig?.useReferee!
        },
        set(value: boolean) {
            console.log('useReferee', value);
            emitter.emit('update:runtimeConfiguration', proto.RuntimeConfig.create({
                ...aiData.state?.runtimeConfig,
                useReferee: value
            }));
        },
    });

    const ignoreInvariants = computed({
        get() {
            return aiData.state?.runtimeConfig?.ignoreInvariants!
        },
        set(value: boolean) {
            emitter.emit('update:runtimeConfiguration', proto.RuntimeConfig.create({
                ...aiData.state?.runtimeConfig,
                ignoreInvariants: value
            }));
        },
    });

    const isLeft = computed({
        get() {
            return aiData.state?.gameSettings?.isLeft!
        },
        set(value: boolean) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...aiData.state?.gameSettings,
                isLeft: value
            }));
        },
    });

    const isYellow = computed({
        get() {
            return aiData.state?.gameSettings?.isYellow!
        },
        set(value: boolean) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...aiData.state?.gameSettings,
                isYellow: value
            }));
        },
    });

    const robotHubMode = computed({
        get() {
            return aiData.state?.gameSettings?.robotHubMode!
        },
        set(value: proto.GameSettings.RobotHubMode) {
            emitter.emit('update:gameSettings', proto.GameSettings.create({
                ...aiData.state?.gameSettings,
                robotHubMode: value
            }));
        },
    });

    const isPaused = computed({
        get() {
            return aiData.state?.isPaused!
        },
        set(value: boolean) {
            emitter.emit('update:pause', value);
        },
    });

    return {
        useReferee,
        ignoreInvariants,
        isLeft,
        isYellow,
        robotHubMode,
        isPaused
    }
};