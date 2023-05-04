import {useGameControllerStore} from "../modules/stores/ai-store";
import {watch} from "vue";
import {useWebSocket} from "@vueuse/core";

import {proto} from "../generated/proto"
import {useGameSettingsStore} from "../modules/stores/game-settings-store";
import {useVisualizationStore} from "../modules/stores/dataStores/visualization-store";
import {useSTPDataStore} from "../modules/stores/dataStores/stp-data-store";
import {useVisionDataStore} from "../modules/stores/dataStores/vision-data-store";
import IGameSettings = proto.IGameSettings;

const sendMsg = (ws: WebSocket, msg: proto.MsgFromInterface) => {
    const buffer = proto.MsgFromInterface.encode(msg).finish();
    ws.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
}

export const useAIClient = (url: string) => {
    const gameSettingsStore = useGameSettingsStore();
    const gameControllerStore = useGameControllerStore();
    const stpDataStore = useSTPDataStore();
    const visionDataStore = useVisionDataStore();
    const visualizationStore = useVisualizationStore();

    const {ws, status, data, send} = useWebSocket(url, {autoReconnect: true});

    // On message received
    watch(data, async (blob) => {
        const messageBuffer = new Uint8Array(await blob.arrayBuffer());
        const envelope = proto.MsgToInterface.decode(messageBuffer);
        switch (envelope.kind) {
            case 'setupMessage':
                console.log('setupMessage', envelope.setupMessage);
                gameControllerStore.processSetupMsg(envelope.setupMessage!);
                break;
            case 'stpStatus':
                stpDataStore.processSTPMsg(envelope.stpStatus!);
                break;
            case 'state':
                visionDataStore.processVisionMsg(envelope.state!);
                break;
            case 'visualizations':
                visualizationStore.pushDrawings(envelope.visualizations!.drawings!);
                visualizationStore.pushMetrics(envelope.visualizations!.metrics!);
                break;
            default:
                console.log('unknown message', envelope);
        }
    });

    // Callbacks to the AI backend
    // Setup watchers for the callbacks to the backend
    watch(gameSettingsStore, (gameSettings: IGameSettings) => {
        sendMsg(ws.value!, proto.MsgFromInterface.create({
            setGameSettings: proto.GameSettings.fromObject(gameSettings),
        }));
    });

    watch(() => gameControllerStore.currentPlay, () => {
        console.log('Sending set play', gameControllerStore.currentPlay);
        sendMsg(ws.value!, proto.MsgFromInterface.create({
            setGameState: proto.GameState.create({
                playName: gameControllerStore.currentPlay.name,
                rulesetName: gameControllerStore.currentPlay.ruleset,
            }),
        }));
    });

    return {
        status,
    }
};