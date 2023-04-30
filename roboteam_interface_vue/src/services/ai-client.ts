// import {useWorldStateStore} from "../modules/stores/world-store";
// import {useSTPStore} from "../modules/stores/stp-store";
import {useAIStore} from "../modules/stores/ai-store";
import {watch} from "vue";
import {useWebSocket} from "@vueuse/core";

import {proto} from "../generated/proto"
import {GameSettingsStore, useGameSettingsStore} from "../modules/stores/game-settings-store";
import {useVisualizationStore} from "../modules/stores/visualization-store";
import RobotHubMode = proto.GameSettings.RobotHubMode;

export const useAIClient = (url: string) => {
    const aiStore = useAIStore();
    const visualizationStore = useVisualizationStore();
    const gameSettingsStore = useGameSettingsStore();

    const {ws, status, data, send} = useWebSocket(url, {autoReconnect: true});

    // On message received
    watch(data, async (blob) => {
        const messageBuffer = new Uint8Array(await blob.arrayBuffer());
        const envelope = proto.MsgToInterface.decode(messageBuffer);
        switch (envelope.kind) {
            case 'setupMessage':
                console.log('setupMessage', envelope.setupMessage);
                aiStore.onConnectMsg(envelope.setupMessage!);

                // Forwards current game settings to the backend
                sendGameSettings(gameSettingsStore);
                break;
            case 'stpStatus':
                aiStore.onSTPStatusMsg(envelope.stpStatus!);
                break;
            case 'state':
                aiStore.onVisionMsg(envelope.state!);
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
    const sendGameSettings = (gameSettings: GameSettingsStore) => {
        const msg = proto.MsgFromInterface.create({
            setGameSettings: proto.GameSettings.create({
                useReferee: gameSettings.useReferee,
                hubMode: gameSettings.hubMode === 'basestation' ? RobotHubMode.BASESTATION : RobotHubMode.SIMULATOR,
                isYellow: gameSettings.team === 'yellow',
                isLeft: gameSettings.side === 'left',
                ignoreInvariants: gameSettings.ignoreInvariants,
            }),
        });

        const buffer = proto.MsgFromInterface.encode(msg).finish();
        ws.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    }

    const forcePlay = () => {
        console.log('sending set play', aiStore.gameController.play, aiStore.gameController.ruleset);
        const msg = proto.MsgFromInterface.create({
            setGameState: proto.GameState.create({
                playName: aiStore.gameController.play,
                rulesetName: aiStore.gameController.ruleset,
            }),
        })

        const buffer = proto.MsgFromInterface.encode(msg).finish();
        ws.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    }

    // Setup watchers for the callbacks to the backend
    watch(gameSettingsStore, sendGameSettings);
    watch(() => aiStore.gameController.play, forcePlay);
    watch(() => aiStore.gameController.ruleset, forcePlay);

    return {
        status,
    }
};