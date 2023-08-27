<script setup lang='ts'>
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { computed, ref } from 'vue'
import { useAiController } from '../../composables/ai-controller'
import { proto, SimulatorCommand } from '../../../generated/proto'
import { VAceEditor } from 'vue3-ace-editor'
import { useDark } from '@vueuse/core'


const
  aiController = useAiController(),
  isDark = useDark(),
  content = ref<string>('')

const disabled = computed(() => aiController.useReferee)

// Move all robots from both teams to side of field
// TODO take field size into account. Currently assumes field size is 12x9
const robotsToSide = () => {
  let teleportRobot = []
  for (const team of [proto.Team.BLUE, proto.Team.YELLOW]) {
    for (let i = 0; i < 16; i++) {
      const x = -6 + i * 0.3
      teleportRobot.push({
        'id': { 'id': i, 'team': team },
        'x': (team === proto.Team.YELLOW ? x : -x),
        'y': -4.6,
        'orientation': 0
      })
    }
  }

  content.value = JSON.stringify({
    'control': {
      'teleportRobot': teleportRobot
    }
  }, null, 2)

  sendSimulatorCommand()
}

// Move ball to center of field
const ballToCenter = () => {
  content.value = JSON.stringify({
    'control': {
      'teleportBall': {
        'x': 0, 'y': 0, 'z': 0, 'vx': 0, 'vy': 0, 'vz': 0
      }
    }
  }, null, 2)

  sendSimulatorCommand()
}

const sendSimulatorCommand = () => {
  aiController.sendSimulatorCommand(JSON.parse(content.value))
}


</script>

<template>
  <div class='mt-4 flex flex-wrap gap-4 max-w-md'>
    <button
      :disabled='disabled'
      class='btn btn-sm btn-secondary gap-2'
      @click='robotsToSide'
    >
      <font-awesome-icon icon='fa-arrows' />
      Move robots to side
    </button>

    <button
      :disabled='disabled'
      class='btn btn-sm btn-secondary gap-2'
      @click="ballToCenter"
    >
      <font-awesome-icon icon='fa-crosshairs' />
      Move ball to center
    </button>
  </div>
  <div class='mt-4 fex gap-4'>
    <div class='text-lg font-medium'>Simulator command content</div>
    <div class='text-sm font-medium mb-2'>The json command has to confirm the ssl simulator command specification (check the proto files)</div>
    <v-ace-editor
      v-model:value='content'
      lang='json'
      :theme="isDark ? 'github_dark' : 'github' "
      min-lines='10'
      :max-lines='Infinity'
      class='mb-2 rounded-lg border dark:border-base-300'
    />
    <button
      :disabled='disabled'
      class='btn btn-sm btn-secondary gap-2'
      @click="sendSimulatorCommand"
    >
      <font-awesome-icon icon='fa-satellite-dish' @click='sendSimulatorCommand' />
      Send command
    </button>

  </div>
</template>