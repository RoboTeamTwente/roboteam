import {defineStore} from 'pinia'
import {useCssVar} from "@vueuse/core";
import {computed, ref, ShallowRef, shallowRef, watch} from "vue";

import {proto} from "../../generated/proto";
import ISTPStatus = proto.ISTPStatus;
import IState = proto.IState;
import ISSL_GeometryData = proto.ISSL_GeometryData;
import IWorld = proto.IWorld;
import ISSL_GeometryFieldSize = proto.ISSL_GeometryFieldSize;
import {useGameSettingsStore} from "./game-settings-store";


type DeepReadonly<T> = T extends Function ? T : T extends object ? { readonly [K in keyof T]: DeepReadonly<T[K]> } : T;
type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>;

export const playHistoryLimit = 10;
export const haltPlayName = 'Halt';

type GameController = {
    play: string,
    ruleset: string,
    availablePlays: string[],
    availableRulesets: string[],
    playHistory: string[],
};

type STPData = {
    currentTick: number,
    latest: ShallowReadonlyRef<ISTPStatus> | null
}

type VisionData = {
    latestWorld: ShallowReadonlyRef<IWorld> | null
    latestField: ShallowReadonlyRef<ISSL_GeometryFieldSize> | null
}

export const useAIStore = defineStore('aiStore', () => {
    const gameSettingStore = useGameSettingsStore();

    const gameController = ref<GameController>({
        play: haltPlayName,
        ruleset: 'default',
        availablePlays: [],
        availableRulesets: [],
        playHistory: [],
    });

    const stpData = ref<STPData>({
        latest: null,
        currentTick: 0,
    });

    const visionData = ref<VisionData>({
        latestField: null,
        latestWorld: null,
    });

    const onConnectMsg = (msg: proto.ISetupMessage) => {
        console.log(msg);
        gameController.value = {
            availablePlays: msg.availablePlays!,
            availableRulesets: msg.availableRulesets!,
            play: 'Halt',
            ruleset: 'default',
            playHistory: [],
        };

        stpData.value = {latest: null, currentTick: 0};
        visionData.value = {
            latestField: null,
            latestWorld: null,
        };
    }

    const onSTPStatusMsg = (msg: proto.ISTPStatus) => {
        stpData.value.latest = msg;
        stpData.value.currentTick++;
    }

    const onVisionMsg = (msg: proto.IState) => {
        if (msg.lastSeenWorld == null) {
            console.warn("Received world is null");
        }

        visionData.value.latestWorld = msg.lastSeenWorld ?? null;

        // Update field geometry only if it has changed
        if (JSON.stringify(msg.field?.field) !== JSON.stringify(visionData.value.latestField)) {
            visionData.value.latestField = msg.field?.field ?? null;
        }
    }

    const haltPlay = () => {
        gameController.value.play = haltPlayName;
        gameController.value.ruleset = 'default';
    };

    const ourRobots = computed(() => {
        return gameSettingStore.team === 'blue'
            ? visionData.value.latestWorld?.blue
            : visionData.value.latestWorld?.yellow;
    });

    return {
        gameController,
        stpData,
        visionData,
        onConnectMsg,
        onSTPStatusMsg,
        onVisionMsg,
        haltPlay,
        ourRobots
    }
});