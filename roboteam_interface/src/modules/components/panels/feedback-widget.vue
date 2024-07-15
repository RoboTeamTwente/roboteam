<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { useUIStore } from '../../stores/ui-store'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { formatFloat } from '../../../utils'

const visionData = useVisionDataStore()
const uiStore = useUIStore()
</script>

<template>
  <div class="grid grid-cols-fluid-12 gap-2">
    <div
      v-if="visionData.ourRobots == null || visionData.ourRobots?.length == 0"
      class="alert alert-sm alert-warning justify-start"
    >
      <font-awesome-icon icon="fa-triangle-exclamation" />
      No data
    </div>

    <template v-for="robot in visionData.ourRobots!" :key="robot.id">
      <div
        class="bg-base-200 p-2 rounded-xl border border-base-300 cursor-pointer"
        :class="{
        'border-base-300 outline outline-2 outline-accent shadow-md': uiStore.isaRobotSelected(robot.id!)
      }"
        v-on:click="uiStore.toggleRobotSelection(robot.id!)"
      >
        <div class="flex flex-wrap gap-1 mb-2">
          <div
            class="badge badge-sm tooltip tooltip-bottom"
            :data-tip="uiStore.robotName(robot.id!)"
          >
            <font-awesome-icon icon="robot" class="w-3 h-3 mr-1" />{{ robot.id }}
          </div>
          <div
            class="badge badge-sm badge-secondary tooltip tooltip-bottom"
            data-tip="Battery level"
          >
            <font-awesome-icon icon="battery" class="h-3 w-3 mr-1" />
            {{ robot.feedbackInfo?.batteryLevel != null ? Number(robot.feedbackInfo.batteryLevel).toFixed(2) : 'N/A' }}
          </div>
          <div
            class="badge badge-sm badge-secondary tooltip tooltip-bottom"
            :data-tip="`Is capacitor charged: ${robot.feedbackInfo?.capacitorIsCharged}`"
            :class="{ 'opacity-50': !robot.feedbackInfo?.capacitorIsCharged }"
          >
            <font-awesome-icon icon="bolt" class="h-3 w-3" />
          </div>
          <div
            class="badge badge-sm badge-secondary tooltip tooltip-bottom"
            :data-tip="`Is ball sensor working: ${robot.feedbackInfo?.ballSensorIsWorking}`"
            :class="{ 'opacity-50': !robot.feedbackInfo?.ballSensorIsWorking }"
          >
            <font-awesome-icon icon="eye" class="h-3 w-3" />
          </div>
        </div>

        <div class="text-no-wrap text-sm font-mono">
          pos:[{{ formatFloat(robot.pos?.x) }}x,{{ formatFloat(robot.pos?.y) }}y]
        </div>
        <div class="text-no-wrap text-sm font-mono">
          vel:[{{ formatFloat(robot.vel?.x) }}x,{{ formatFloat(robot.vel?.y) }}y]
        </div>
        <div class="text-no-wrap text-sm font-mono">yaw: {{ formatFloat(robot.yaw) }}rad</div>
        <div class="text-no-wrap text-sm font-mono">w: {{ formatFloat(robot.w) }}rad/s</div>
        <div class="text-no-wrap text-sm font-mono">
          Ball detected: {{ robot.feedbackInfo?.ballSensorSeesBall }}
        </div>
        <div class="text-no-wrap text-sm font-mono">
          Dribbler sees ball: {{ robot.feedbackInfo?.dribblerSeesBall }}
        </div>
      </div>
    </template>
  </div>
</template>
