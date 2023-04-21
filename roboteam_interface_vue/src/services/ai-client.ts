import {useWorldStateStore} from "../modules/stores/world-store";
import {useSTPStore} from "../modules/stores/stp-store";
import {useAIStore} from "../modules/stores/ai-store";
import {watch} from "vue";
import {useWebSocket} from "@vueuse/core";

import {proto} from "../generated/proto"
import {GameSettingsStore, useGameSettingsStore} from "../modules/stores/game-settings-store";
import RobotHubMode = proto.SetGameSettings.RobotHubMode;

export const useAIClient = (url: string) => {
    const worldStore = useWorldStateStore();
    const stpStore = useSTPStore();
    const aiStore = useAIStore();
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
                aiStore.availablePlays = envelope.setupMessage!.availablePlays ?? [];
                aiStore.availableRulesets = envelope.setupMessage!.availableRulesets ?? [];

                // Forwards current game settings to the backend
                sendGameSettings(gameSettingsStore);
                break;
            case 'stpStatus':
                stpStore.pushNewStatus(envelope.stpStatus!);
                break;
            case 'state':
                if (envelope.state!.lastSeenWorld === undefined) {
                    console.log('no last seen world');
                    return;
                }

                worldStore.pushNewState(envelope.state!!);
                break;
            default:
                console.log('unknown message', envelope);
        }
    });

    const forcePlay = () => {
        console.log('sending set play', aiStore.play, aiStore.ruleset);
        const msg = proto.InterfaceMessageEnvelope.create({
            setPlay: proto.SetPlay.create({
                playName: aiStore.play,
                rulesetName: aiStore.ruleset,
            }),
        })

        const buffer = proto.InterfaceMessageEnvelope.encode(msg).finish();
        ws.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    }

    watch(() => aiStore.play, forcePlay);
    watch(() => aiStore.ruleset, forcePlay);

    return {
        status,
    }
};