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
  const knownBatteryLevels = shallowRef<{ [robotId: number]: number }>({});

  const $reset = () => {
    latestWorld.value = null
    latestField.value = null
    knownBatteryLevels.value = {}
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

    for (const robot in aiData.state?.gameSettings?.isYellow ? latestWorld.value?.yellow : latestWorld.value?.blue) {
      if ( robot.feedbackInfo?.batteryLevel != null ) {
        
      }
    }

    if (ourRobots) {
      for (const robot of ourRobots) {
        const id = robot.id;
        const batteryLevel = robot.feedbackInfo?.batteryLevel;
        // if (id == 1) batteryLevel == null
        // if (id == 4) batteryLevel == 22
        // if (id == 8) batteryLevel == 25
        // if (id == 9) batteryLevel == 30

        if (batteryLevel !== null && batteryLevel !== undefined && batteryLevel >= 18 && batteryLevel <= 25.2) {
          knownBatteryLevels.value[id] = batteryLevel;
        } else if (batteryLevel === null || batteryLevel < 18) {
          robot.feedbackInfo = robot.feedbackInfo || {};
          robot.feedbackInfo.batteryLevel = knownBatteryLevels.value[id];
        }
      }
    }
  }

  const ourRobots = computed(() => {
    return aiData.state?.gameSettings?.isYellow
      ? latestWorld.value?.yellow
      : latestWorld.value?.blue
  })

  return {
    latestWorld: readonly(latestWorld),
    latestField: readonly(latestField),
    $reset,
    processVisionMsg,
    ourRobots
  }
})
