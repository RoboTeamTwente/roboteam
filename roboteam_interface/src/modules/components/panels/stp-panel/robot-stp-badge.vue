<script setup lang="ts">
import { computed } from 'vue'
import { proto } from '../../../../generated/proto'
import STPRobot = proto.STPStatus.STPRobot

const props = defineProps<{
  status: STPRobot.Status | 'Unknown'
  name: string | 'Unknown'
}>()

const statusIcon = computed(() => {
  return {
    1: 'fa-circle-check',
    3: 'fa-arrows-rotate',
    2: 'fa-triangle-exclamation',
    0: 'fa-hourglass-start',
    Unknown: 'fa-question'
  }[props.status]
})
</script>

<template>
  <div class="kbd gap-2 text-sm md:text-base flex items-center justify-center">
    {{ props.name }}
    <font-awesome-icon
      :icon="statusIcon"
      :spin="props.status === 3"
      class="w-3 h-3 text-secondary"
      :class="{
        'text-success': props.status === 1,
        'text-error': props.status === 2
      }"
    />
  </div>
</template>
