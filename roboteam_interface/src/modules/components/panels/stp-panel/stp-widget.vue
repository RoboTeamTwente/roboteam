<script setup lang="ts">
import RobotStpBadge from './robot-stp-badge.vue'
import { useUIStore } from '../../../stores/ui-store'
import { useSTPDataStore } from '../../../stores/data-stores/stp-data-store'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'

const stpData = useSTPDataStore()
const uiStore = useUIStore()
</script>

<template>
  <div class="grid grid-cols-fluid-10 gap-2 auto-cols-max">
    <div v-if="stpData.latest == null" class="alert alert-sm alert-warning justify-start">
      <font-awesome-icon icon="fa-triangle-exclamation" />
      No data
    </div>

    <template v-for="robot in stpData.latest?.robots" :key="robot.id">
      <div
        class="flex flex-col gap-1 justify-center text-center bg-base-200 rounded-xl p-2 self-start border dark:border-base-300 cursor-pointer"
        :class="{
        'border-base-300 outline outline-2 outline-accent shadow-md': uiStore.isaRobotSelected(robot.id!)
      }"
        v-on:click="uiStore.toggleRobotSelection(robot.id!)"
      >
        <div class="kbd gap-2 text-sm md:text-base flex justify-center">
          <font-awesome-icon icon="robot" />
          {{ robot.id }} {{ uiStore.robotName(robot.id ?? -1) }}
        </div>
        <robot-stp-badge
          :name="robot.role?.name ?? 'Unknown'"
          :status="robot.role?.status ?? 'Unknown'"
        />
        <robot-stp-badge
          v-if="robot.tactic != null"
          :name="robot.tactic!.name!"
          :status="robot.tactic!.status!"
        />
        <robot-stp-badge
          v-if="robot.skill != null"
          :name="robot.skill!.name!"
          :status="robot.skill!.status!"
        />
      </div>
    </template>
  </div>
</template>
