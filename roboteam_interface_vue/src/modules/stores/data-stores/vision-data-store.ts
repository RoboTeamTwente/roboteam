import { proto } from '../../../generated/proto'
import IWorld = proto.IWorld
import ISSL_GeometryFieldSize = proto.ISSL_GeometryFieldSize
import { defineStore } from 'pinia'
import { computed, readonly, shallowRef } from 'vue'
import { useAIDataStore } from './ai-data-store'

export const useVisionDataStore = defineStore('visionDataStore', () => {
  const aiData = useAIDataStore()

  const latestWorld = shallowRef<IWorld | null>(null)
  const latestField = shallowRef<ISSL_GeometryFieldSize | null>(null)

  const $reset = () => {
    latestWorld.value = null
    latestField.value = null
  }

  const processVisionMsg = (msg: proto.IState) => {
    if (msg.lastSeenWorld == null) {
      console.warn('Received world is null')
    }

    latestWorld.value = msg.lastSeenWorld ?? null

    // Update field geometry only if it has changed
    if (JSON.stringify(msg.field?.field) !== JSON.stringify(latestField.value)) {
      latestField.value = msg.field?.field ?? null
    }
  }

  const ourRobots = computed(() => {
    return aiData.state?.gameSettings?.isYellow
      ? latestWorld.value?.blue
      : latestWorld.value?.yellow
  })

  return {
    latestWorld: readonly(latestWorld),
    latestField: readonly(latestField),
    $reset,
    processVisionMsg,
    ourRobots
  }
})
