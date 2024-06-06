import { computed } from 'vue'
import { ISimulatorCommand, proto } from '../../generated/proto'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'
import { defineStore } from 'pinia'
import { useSTPDataStore } from '../stores/data-stores/stp-data-store'
import { useVisionDataStore } from '../stores/data-stores/vision-data-store'
import { useVisualizationStore } from '../stores/data-stores/visualization-store'
import { useProtoWebSocket } from './proto-websocket'

export const useAiController = defineStore('aiController', () => {
  // Dependencies
  const stpData = useSTPDataStore(),
    visionData = useVisionDataStore(),
    visualizationData = useVisualizationStore(),
    aiData = useAIDataStore()

  const { status, send, open, close } = useProtoWebSocket((message) =>
    ({
      aiState: () => aiData.updateStateFromProto(message.aiState!),
      stpStatus: () => stpData.processSTPMsg(message.stpStatus!),
      state: () => visionData.processVisionMsg(message.state!),
      visualizations: () => {
        visualizationData.pushDrawings(message.visualizations!.drawings!)
        visualizationData.pushMetrics(message.visualizations!.metrics!)
      }
    }[message.kind!]())
  )

  // Actions
  const setBallPos = (x: number, y: number) => send('setBallPos', { x, y })

  const sendSimulatorCommand = (command: ISimulatorCommand) => send('simulatorCommand', command)

  // Writable computed properties
  const useReferee = computed({
      get: () => aiData.state?.runtimeConfig?.useReferee!,
      set: (value: boolean) =>
        send('setRuntimeConfig', {
          ...aiData.state?.runtimeConfig,
          useReferee: value
        })
    }),
    ignoreInvariants = computed({
      get: () => aiData.state?.runtimeConfig?.ignoreInvariants!,
      set: (value: boolean) =>
        send('setRuntimeConfig', {
          ...aiData.state?.runtimeConfig,
          ignoreInvariants: value
        })
    }),
    isLeft = computed({
      get: () => aiData.state?.gameSettings?.isLeft!,
      set: (value: boolean) =>
        send('setGameSettings', {
          ...aiData.state?.gameSettings,
          isLeft: value
        })
    }),
    isYellow = computed({
      get: () => aiData.state?.gameSettings?.isYellow!,
      set: (value: boolean) =>
        send('setGameSettings', {
          ...aiData.state?.gameSettings,
          isYellow: value
        })
    }),
    robotHubMode = computed({
      get: () => aiData.state?.gameSettings?.robotHubMode!,
      set: (value: proto.GameSettings.RobotHubMode) =>
        send('setGameSettings', {
          ...aiData.state?.gameSettings,
          robotHubMode: value
        })
    }),
    isPaused = computed({
      get: () => aiData.state?.isPaused!,
      set: (value: boolean) => send('pauseAi', value)
    }),
    currentPlayName = computed({
      get: () => stpData.latest?.currentPlay!.playName!,
      set: (value: string) =>
        send('setPlay', {
          ...stpData.latest?.currentPlay!,
          rulesetName: 'default',
          playName: value
        })
    }),
    currentRuleset = computed({
      get: () => stpData.latest?.currentPlay!.rulesetName!,
      set: (value: string) =>
        send('setPlay', {
          ...stpData.latest?.currentPlay!,
          rulesetName: value
        })
    })

  return {
    open,
    close,
    status,
    setBallPos,
    useReferee,
    ignoreInvariants,
    isLeft,
    isYellow,
    robotHubMode,
    isPaused,
    currentPlayName,
    currentRuleset,
    sendSimulatorCommand
  }
})
