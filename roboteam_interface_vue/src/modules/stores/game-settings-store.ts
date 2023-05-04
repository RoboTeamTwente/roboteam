import {defineStore} from 'pinia'
import {Colors} from "../field/field-objects";
import {computed, reactive} from "vue";
import {proto} from "../../generated/proto";
import IGameSettings = proto.IGameSettings;


export const useGameSettingsStore = defineStore('gameSettingsStore', () => {
    const value = reactive<IGameSettings>({})
    const goalColor = computed(() => {
        return value.isYellow
            ? {leftGoal: Colors.yellow, rightGoal: Colors.blue}
            : {leftGoal: Colors.blue, rightGoal: Colors.yellow};
    });

    const fieldOrientation = computed(() => {
        return value.isLeft
            ? {x: 1, y: -1, angle: 0}
            : {x: -1, y: 1, angle: 180};
    });

    return {
        ...value,
        goalColor,
        fieldOrientation,
    }
})