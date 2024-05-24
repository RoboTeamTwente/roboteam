import { defineStore } from 'pinia'
import { proto } from '../../../generated/proto'
import { computed, readonly, shallowRef } from 'vue'
import IAIState = proto.IAIState

export const useAIDataStore = defineStore('aiDataStore', () => {
  const state = shallowRef<IAIState | null>({})

  // Actions
  const updateStateFromProto = (msg: proto.IAIState) => {
      console.log('updateStateFromProto', msg)
      state.value = msg
    },
    $reset = () => {
      state.value = null
    }

  // Getters
  const fieldOrientation = computed(() => {
    return state.value?.gameSettings?.isLeft ? { x: 1, y: -1, yaw: 0 } : { x: -1, y: 1, yaw: 180 }
  })

  return {
    state: readonly(state),
    updateStateFromProto,
    $reset,
    fieldOrientation
  }
})
