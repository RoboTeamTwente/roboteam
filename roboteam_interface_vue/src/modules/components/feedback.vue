<script setup lang="ts">
import {FontAwesomeIcon} from "@fortawesome/vue-fontawesome";
import {useUIStore} from "../stores/ui-store";
import {useAIStore} from "../stores/ai-store";

const aiStore = useAIStore();
const uiStore = useUIStore();

const formatFloat = (pos?: number | null): string => {
  if (pos === undefined || pos === null) {
    return '0.00';
  }

  const value = Math.abs(pos).toFixed(2);
  const sign = pos > -0.0001 ? '+' : '-';
  return `${sign}${value}`;
}

</script>

<template>
  <div class="grid grid-cols-fluid-12  gap-2 ">
    <template v-for="(robot, id) in aiStore.ourRobots" :key="id">
      <div class="bg-base-200 p-2 rounded-xl border border-base-300" :class="{
        'outline outline-2 outline-accent/50': uiStore.isaRobotSelected(robot.id!)
      }">
        <div class="flex flex-wrap gap-1 mb-2">
          <div class="badge badge-sm tooltip tooltip-bottom" :data-tip="uiStore.robotName(id)">
            <font-awesome-icon icon="robot" class="w-3 h-3 mr-1"/>{{id}}
          </div>
          <div class="badge badge-sm badge-secondary tooltip tooltip-bottom" data-tip="Battery level">
            <font-awesome-icon icon="battery" class="h-3 w-3 mr-1"/> {{robot.feedbackInfo?.batteryLevel}}
          </div>
          <div class="badge badge-sm badge-secondary tooltip tooltip-bottom text-no-wrap" data-tip="Signal strength">
            <font-awesome-icon icon="signal" class="h-3 w-3 mr-1"/> {{robot.feedbackInfo?.signalStrength}}
          </div>
          <div class="badge badge-sm badge-secondary tooltip tooltip-bottom"
               :data-tip="`Is capacitor charged: ${robot.feedbackInfo?.capacitorIsCharged}`"
               :class="{'opacity-50': !robot.feedbackInfo?.capacitorIsCharged,}">
            <font-awesome-icon icon="bolt" class="h-3 w-3"/>
          </div>
            <div class="badge badge-sm badge-secondary tooltip tooltip-bottom"
                 :data-tip="`Is ball sensor working: ${robot.feedbackInfo?.ballSensorIsWorking}`"
                 :class="{'opacity-50': !robot.feedbackInfo?.ballSensorIsWorking,}">
              <font-awesome-icon icon="eye" class="h-3 w-3"/>
            </div>
        </div>

        <div class="text-no-wrap text-sm font-mono">pos:[{{ formatFloat(robot.pos?.x) }}x,{{
            formatFloat(robot.pos?.y)
          }}y]</div>
        <div class="text-no-wrap text-sm font-mono">vel:[{{ formatFloat(robot.vel?.x) }}x,{{
            formatFloat(robot.vel?.y)
          }}y]
        </div>
        <div class="text-no-wrap text-sm font-mono">angle: {{ formatFloat(robot.angle) }}rad</div>
        <div class="text-no-wrap text-sm font-mono">w: {{ formatFloat(robot.w)}}rad/s</div>
        <div class="text-no-wrap text-sm font-mono">Ball detected: {{robot.feedbackInfo?.ballSensorSeesBall}} </div>
        <div class="text-no-wrap text-sm font-mono">Ball position: {{robot.feedbackInfo?.ballPosition}} </div>
        <div class="text-no-wrap text-sm font-mono">Dribbler sees ball: {{robot.feedbackInfo?.dribblerSeesBall}} </div>
      </div>
    </template>
  </div>
</template>
