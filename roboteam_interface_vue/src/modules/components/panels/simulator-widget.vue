<script setup lang="ts">
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { computed, toRaw } from 'vue'
import { useAiController } from '../../composables/ai-controller'
import { proto } from '../../../generated/proto'

const aiController = useAiController()
const disabled = computed(() => aiController.useReferee)

// Move all robots from both teams to side of field
// TODO take field size into account. Currently assumes field size is 12x9
const robotsToSide = (() => {
  let teleportRobot = []
  for (const team of [proto.Team.BLUE, proto.Team.YELLOW]) {
    for (let i = 0; i < 16; i++) {
      const x = -6 + i * 0.3
      teleportRobot.push({
        id: { id: i, team: team },
        x: team === proto.Team.YELLOW ? x : -x,
        y: -4.6,
        orientation: 0
      })
    }
  }
  return {
    control: {
      teleportRobot: teleportRobot
    }
  }
})()

// Move ball to center of field
const ballToCenter = {
  control: {
    teleportBall: {
      x: 0,
      y: 0,
      z: 0,
      vx: 0,
      vy: 0,
      vz: 0
    }
  }
}

const commandMap = new Map<string, Object>([
  ['robotsToSide', robotsToSide],
  ['ballToCenter', ballToCenter]
])

const sendSimulatorCommand = (command: string) => {
  aiController.sendSimulatorCommand(commandMap.get(command)!)
}
</script>

<template>
  <div class="mt-4 flex flex-col gap-4 max-w-md">
    <button
      :disabled="disabled"
      class="btn btn-sm btn-secondary gap-2"
      @click="sendSimulatorCommand('robotsToSide')"
    >
      <font-awesome-icon icon="fa-arrows" />
      Move robots to side
    </button>

    <button
      :disabled="disabled"
      class="btn btn-sm btn-secondary gap-2"
      @click="sendSimulatorCommand('ballToCenter')"
    >
      <font-awesome-icon icon="fa-crosshairs" />
      Move ball to center
    </button>
  </div>
</template>
