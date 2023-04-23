// import {useWorldStateStore} from "../modules/stores/world-store";
// import {useSTPStore} from "../modules/stores/stp-store";
import {useAIStore} from "../modules/stores/ai-store";
import {watch} from "vue";
import {useWebSocket} from "@vueuse/core";

import {proto} from "../generated/proto"
import {GameSettingsStore, useGameSettingsStore} from "../modules/stores/game-settings-store";
import RobotHubMode = proto.SetGameSettings.RobotHubMode;
import {useVisualizationStore} from "../modules/stores/visualization-store";

export const useAIClient = (url: string) => {
    const aiStore = useAIStore();
    const visualizerStore = useVisualizationStore();
    const gameSettingsStore = useGameSettingsStore();

    const {ws, status, data, send} = useWebSocket(url, {
        autoReconnect: true,
        // protocols: ['binary'],
    });

    watch(gameSettingsStore, (state) => sendGameSettings(state));
    const sendGameSettings = (gameSettings: GameSettingsStore) => {
        const msg = proto.InterfaceMessageEnvelope.create({
            setGameSettings: proto.SetGameSettings.create({
                useReferee: gameSettings.useReferee,
                hubMode: gameSettings.hubMode === 'basestation' ? RobotHubMode.BASESTATION : RobotHubMode.SIMULATOR,
                isYellow: gameSettings.team === 'yellow',
                isLeft: gameSettings.side === 'left',
                ignoreInvariants: gameSettings.ignoreInvariants,
            }),
        });

        const buffer = proto.InterfaceMessageEnvelope.encode(msg).finish();
        ws.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    }


    watch(data, async (blob) => {
        const messageBuffer = new Uint8Array(await blob.arrayBuffer());
        const envelope = proto.MessageEnvelope.decode(messageBuffer);
        switch (envelope.kind) {
            case 'setupMessage':
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
            case 'drawing':
                visualizerStore.push(envelope.drawing!);
                break;
            default:
                console.log('unknown message', envelope);
        }
    });

    const forcePlay = () => {
        console.log('sending set play', aiStore.gameController.play, aiStore.gameController.ruleset);
        const msg = proto.InterfaceMessageEnvelope.create({
            setPlay: proto.SetPlay.create({
                playName: aiStore.gameController.play,
                rulesetName: aiStore.gameController.ruleset,
            }),
        })

        const buffer = proto.InterfaceMessageEnvelope.encode(msg).finish();
        ws.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    }

    watch(() => aiStore.gameController.play, forcePlay);
    watch(() => aiStore.gameController.ruleset, forcePlay);

    return {
        status,
    }
};