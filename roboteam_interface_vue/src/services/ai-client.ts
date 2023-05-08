import {useGameControllerStore} from "../modules/stores/ai-store";
import {watch} from "vue";
import {watchDeep} from "@vueuse/core";

import {proto} from "../generated/proto"
import {useGameSettingsStore} from "../modules/stores/game-settings-store";
import {useVisualizationStore} from "../modules/stores/dataStores/visualization-store";
import {useSTPDataStore} from "../modules/stores/dataStores/stp-data-store";
import {useVisionDataStore} from "../modules/stores/dataStores/vision-data-store";
import IGameSettings = proto.IGameSettings;
import {useProtoWebSocket} from "../utils";


export const useAIClient = (url: string) => {
    const gameSettingsStore = useGameSettingsStore();
    const gameControllerStore = useGameControllerStore();
    const stpDataStore = useSTPDataStore();
    const visionDataStore = useVisionDataStore();
    const visualizationStore = useVisualizationStore();

    // const {status, data, send} = useWebSocket(url, {autoReconnect: true});
    const {status, data, send, debounce} = useProtoWebSocket(url, {autoReconnect: true}, {
        gameSettings: false,
        currentPlay: false,
    });

    // On message received
    watch(data, (message) => {
        switch (message.kind) {
            case 'setupMessage':
                debounce.gameSettings = true;
                debounce.currentPlay = true;

                console.log('setupMessage', message.setupMessage);
                gameControllerStore.processSetupMsg(message.setupMessage!);
                gameSettingsStore.processSetupMsg(message.setupMessage!);
                break;
            case 'stpStatus':
                stpDataStore.processSTPMsg(message.stpStatus!);
                break;
            case 'state':
                visionDataStore.processVisionMsg(message.state!);
                break;
            case 'visualizations':
                visualizationStore.pushDrawings(message.visualizations!.drawings!);
                visualizationStore.pushMetrics(message.visualizations!.metrics!);
                break;
            default:
                console.log('unknown message', message);
        }
    });

    // Callbacks to the AI backend
    // Setup watchers for the callbacks to the backend
    watch(gameSettingsStore, (gameSettings: IGameSettings) => send(proto.MsgFromInterface.create({
        setGameSettings: proto.GameSettings.fromObject(gameSettings),
    }), 'gameSettings'));

    watchDeep(() => gameControllerStore.currentPlay, () => send(proto.MsgFromInterface.create({
        setGameState: proto.GameState.create({
            playName: gameControllerStore.currentPlay.name,
            rulesetName: gameControllerStore.currentPlay.ruleset,
        }),
    }), 'currentPlay'));

    return {
        status,
    }
};