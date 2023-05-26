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
    keeperId: number,
}

type AvailablePlays = {
    plays: string[],
    rule_sets: string[],
}

export const useGameControllerStore = defineStore('gameController', () => {
    const stpDataStore = useSTPDataStore();
    const visionDataStore = useVisionDataStore();

    // State
    // const isAIPaused = ref(false);
    // const useReferee = ref(false);
    // const ignoreInvariants = ref(false);

    // const currentPlay = ref<CurrentPlay>({name: haltPlayName, ruleset: 'default', keeperId: 0});
    // const availablePlays = ref<AvailablePlays>({
    //     plays: [],
    //     rule_sets: [],
    // });

    // Actions
    // const toggleAIPaused = () => isAIPaused.value = !isAIPaused.value;

    const updateFromProtoMessage = (msg: proto.IAIState) => {
        // isAIPaused.value = msg.isPaused ?? false;
        // useReferee.value = msg.runtimeConfig!.useReferee!;
        // ignoreInvariants.value = msg.runtimeConfig!.ignoreInvariants!;
        //
        // currentPlay.value = {
        //     name: msg.currentPlay?.playName!,
        //     ruleset: msg.currentPlay?.rulesetName!,
        //     keeperId: msg.currentPlay?.keeperId!,
        // };
        //
        // availablePlays.value = {
        //     plays: msg.plays!,
        //     rule_sets: msg.ruleSets!,
        // }

        stpDataStore.$reset();
        visionDataStore.$reset();
    }

    // const haltPlay = () => currentPlay.value = {
    //     ...currentPlay.value,
    //     name: haltPlayName,
    //     ruleset: 'default',
    // };

    // const resetPlay = async () => {
    //     const playToRest = currentPlay.value;
    //     haltPlay();
    //     await sleep(10);
    //     currentPlay.value = playToRest;
    // }

    return {
        // useReferee,
        // isAIPaused,
        // ignoreInvariants,
        // currentPlay,
        // availablePlays,
        // toggleAIPaused,
        updateFromProtoMessage,
        // haltPlay,
        // resetPlay,
    }
});