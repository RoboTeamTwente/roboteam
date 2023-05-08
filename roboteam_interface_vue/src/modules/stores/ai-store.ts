import {defineStore} from 'pinia'
import {computed, ref, watch} from "vue";

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
    const useReferee = ref(false);
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
        useReferee.value = msg.aiSettings!.useReferee!;
        ignoreInvariants.value = msg.aiSettings!.useReferee!;

        availablePlays.value = {
            plays: msg.availablePlays!,
            rule_sets: msg.availableRulesets!,
        }

        stpDataStore.$reset();
        visionDataStore.$reset();

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
        useReferee,
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