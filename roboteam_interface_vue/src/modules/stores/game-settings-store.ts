import {defineStore} from 'pinia'
import {Colors} from "../field/field-objects";
import {computed, ref} from "vue";
import {proto} from "../../generated/proto";
import RobotHubMode = proto.GameSettings.RobotHubMode;


export const useGameSettingsStore = defineStore('gameSettingsStore', () => {
    const isYellow = ref(false);
    const isLeft = ref(false);
    const robotHubMode = ref<RobotHubMode>(RobotHubMode.UNKNOWN);

    const goalColor = computed(() => {
        return isYellow.value
            ? {leftGoal: Colors.yellow, rightGoal: Colors.blue}
            : {leftGoal: Colors.blue, rightGoal: Colors.yellow};
    });

    const processSetupMsg = (msg: proto.ISetupMessage) => {
        isYellow.value = msg.gameSettings!.isYellow!;
        isLeft.value = msg.gameSettings!.isLeft!;
        robotHubMode.value = msg.gameSettings!.robotHubMode!;

    }

    const fieldOrientation = computed(() => {
        return isLeft.value
            ? {x: 1, y: -1, angle: 0}
            : {x: -1, y: 1, angle: 180};
    });

    return {
        isYellow,
        isLeft,
        robotHubMode,
        goalColor,
        processSetupMsg,
        fieldOrientation,
    }
})