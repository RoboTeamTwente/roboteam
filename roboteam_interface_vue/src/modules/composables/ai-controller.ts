import { computed } from 'vue'
import { aiEmitter } from '../../services/events'
import { proto } from '../../generated/proto'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'

export const useAiController = () => {
  const aiData = useAIDataStore()

  const useReferee = computed({
    get() {
      return aiData.state?.runtimeConfig?.useReferee!
    },
    set(value: boolean) {
      console.log('useReferee', value)
      aiEmitter.emit(
        'update:runtimeConfiguration',
        proto.RuntimeConfig.create({
          ...aiData.state?.runtimeConfig,
          useReferee: value
        })
      )
    }
  })

  const ignoreInvariants = computed({
    get() {
      return aiData.state?.runtimeConfig?.ignoreInvariants!
    },
    set(value: boolean) {
      aiEmitter.emit(
        'update:runtimeConfiguration',
        proto.RuntimeConfig.create({
          ...aiData.state?.runtimeConfig,
          ignoreInvariants: value
        })
      )
    }
  })

  const isLeft = computed({
    get() {
      return aiData.state?.gameSettings?.isLeft!
    },
    set(value: boolean) {
      aiEmitter.emit(
        'update:gameSettings',
        proto.GameSettings.create({
          ...aiData.state?.gameSettings,
          isLeft: value
        })
      )
    }
  })

  const isYellow = computed({
    get() {
      return aiData.state?.gameSettings?.isYellow!
    },
    set(value: boolean) {
      aiEmitter.emit(
        'update:gameSettings',
        proto.GameSettings.create({
          ...aiData.state?.gameSettings,
          isYellow: value
        })
      )
    }
  })

  const robotHubMode = computed({
    get() {
      return aiData.state?.gameSettings?.robotHubMode!
    },
    set(value: proto.GameSettings.RobotHubMode) {
      aiEmitter.emit(
        'update:gameSettings',
        proto.GameSettings.create({
          ...aiData.state?.gameSettings,
          robotHubMode: value
        })
      )
    }
  })

  const isPaused = computed({
    get() {
      return aiData.state?.isPaused!
    },
    set(value: boolean) {
      aiEmitter.emit('update:pause', value)
    }
  })

  return {
    useReferee,
    ignoreInvariants,
    isLeft,
    isYellow,
    robotHubMode,
    isPaused
  }
}
