import {defineStore} from 'pinia'
import {computed, ref} from "vue";

import {proto} from "../../generated/proto";
import {useSTPDataStore} from "./dataStores/stp-data-store";
import {useVisionDataStore} from "./dataStores/vision-data-store";
import {sleep} from "../../utils";

export const haltPlayName = 'Halt';


type CurrentPlay = {
    name: string,
    ruleset: string,
}

type AvailablePlays = {
    plays: string[],
    rule_sets: string[],
}

export const useGameControllerStore = defineStore('gameController', () => {
    const stpDataStore = useSTPDataStore();
    const visionDataStore = useVisionDataStore();

    // State
    const isAIPaused = ref(false);
    const ignoreInvariants = ref(false);

    const currentPlay = ref<CurrentPlay>({name: haltPlayName, ruleset: 'default'});
    const availablePlays = ref<AvailablePlays>({
        plays: [],
        rule_sets: [],
    });

    // Actions
    const toggleAIPaused = () => isAIPaused.value = !isAIPaused.value;

    const processSetupMsg = (msg: proto.ISetupMessage) => {
        haltPlay();
        availablePlays.value = {
            plays: msg.availablePlays!,
            rule_sets: msg.availableRulesets!,
        }

        stpDataStore.currentTick = 0;
        stpDataStore.latest = null;

        visionDataStore.latestField = null;
        visionDataStore.latestWorld = null;

        isAIPaused.value = msg.isPaused ?? false;
    }

    const haltPlay = () => currentPlay.value = {
        name: haltPlayName,
        ruleset: 'default',
    };

    const resetPlay = async () => {
        const playToRest = currentPlay.value;
        haltPlay();
        await sleep(10);
        currentPlay.value = playToRest;
    }

    return {
        isAIPaused,
        ignoreInvariants,
        currentPlay,
        availablePlays,
        toggleAIPaused,
        processSetupMsg,
        haltPlay,
        resetPlay,
    }
});