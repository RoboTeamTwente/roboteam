<script setup lang='ts'>
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { computed, ref } from 'vue'
import { useAiController } from '../../composables/ai-controller'
import { proto } from '../../../generated/proto'
import { VAceEditor } from 'vue3-ace-editor'
import { useDark } from '@vueuse/core'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'


const
  aiController = useAiController(),
  visionData = useVisionDataStore(),
  isDark = useDark(),
  content = ref<string>('')

const disabled = computed(() => aiController.useReferee)

// Move all robots from both teams to side of field
const robotsToSide = () => {
  let teleportRobot = []
  const fieldLength = visionData.latestField!.fieldLength / 1000
  const fieldWidth = visionData.latestField!.fieldWidth / 1000 + 0.3
  const startX = -fieldLength / 2

  for (const team of [proto.Team.BLUE, proto.Team.YELLOW]) {
    for (let i = 0; i < 16; i++) {
      const x = startX + (i * fieldWidth / 15) * 0.5
      const sideMultiplier = (team === proto.Team.YELLOW) ? 1 : -1
      teleportRobot.push({
        'id': { 'id': i, 'team': team },
        'x': x * sideMultiplier,
        'y': -fieldWidth / 2,
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

const recordPosition = () => {
  if (!visionData.latestWorld) {
    return
  }

  const ballPos = {
    'x': visionData.latestWorld.ball!.pos!.x,
    'y': visionData.latestWorld.ball!.pos!.y,
    'z': visionData.latestWorld.ball!.z,
    'vx': visionData.latestWorld.ball!.vel?.x,
    'vy': visionData.latestWorld.ball!.vel?.y,
    'vz': visionData.latestWorld.ball!.zVel
  }

  let robots = []
  for (const robot of visionData.latestWorld.blue!) {
    robots.push({
      'id': { 'id': robot.id, 'team': proto.Team.BLUE },
      'x': robot.pos!.x,
      'y': robot.pos!.y,
      'orientation': robot.angle
    })
  }

  for (const robot of visionData.latestWorld.yellow!) {
    robots.push({
      'id': { 'id': robot.id, 'team': proto.Team.YELLOW },
      'x': robot.pos!.x,
      'y': robot.pos!.y,
      'orientation': robot.angle
    })
  }

  content.value = JSON.stringify({
    'control': {
      'teleportBall': ballPos,
      'teleportRobot': robots
    }
  }, null, 2)

  sendSimulatorCommand()
}

const sendSimulatorCommand = () => {
  aiController.sendSimulatorCommand(JSON.parse(content.value))
}


</script>

<template>
  <div class='mt-2 text-lg font-medium'>Quick actions</div>
  <div class='flex flex-wrap gap-4'>
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
      @click='ballToCenter'
    >
      <font-awesome-icon icon='fa-crosshairs' />
      Move ball to center
    </button>

    <button
      :disabled='disabled'
      class='btn btn-sm btn-secondary btn-outline gap-2'
      @click='recordPosition'
    >
      <font-awesome-icon icon='fa-save' />
      Record current position
    </button>
  </div>
  <div class='mt-4 fex gap-4'>
    <div class='text-lg font-medium'>Simulator command content</div>
    <div class='text-sm font-medium mb-2'>The json command has to confirm the ssl simulator command specification (check
      the proto files)
    </div>
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
      @click='sendSimulatorCommand'
    >
      <font-awesome-icon icon='fa-satellite-dish' />
      Send command
    </button>

  </div>
</template>