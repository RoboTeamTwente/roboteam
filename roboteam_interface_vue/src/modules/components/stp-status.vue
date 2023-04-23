<script setup lang="ts">
import RobotStpBadge from "./robot-stp-badge.vue";
import {useUIStore} from "../stores/ui-store";
import {useAIStore} from "../stores/ai-store";

const aiStore = useAIStore()
const uiStore = useUIStore();
</script>

<template>
  <div class="grid grid-cols-fluid-10 gap-2 auto-cols-max">
    <template v-for="robot in aiStore.$state.stpData.latest?.robots" :key="robot.id">
      <div class="flex flex-col gap-1 justify-center text-center bg-base-200 rounded-xl p-2 self-start border" :class="{
        'bg-base-300 shadow-sm': uiStore.isaRobotSelected(robot.id)
      }">
        <div class="kbd gap-2 text-sm md:text-base flex justify-center">
          <font-awesome-icon icon="robot"/> {{robot.id}}
        </div>
        <robot-stp-badge :name="robot.role?.name" :status="robot.role?.status"/>
        <robot-stp-badge :name="robot.tactic?.name" :status="robot.tactic?.status"/>
        <robot-stp-badge :name="robot.skill?.name" :status="robot.skill?.status"/>
      </div>

    </template>
  </div>
</template>
