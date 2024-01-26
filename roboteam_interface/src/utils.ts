import { ShallowRef } from 'vue'
import { proto } from './generated/proto'
import SSL_Referee = proto.SSL_Referee
import Stage = proto.SSL_Referee.Stage

export type NoUndefinedField<T> = { [P in keyof T]-?: NoUndefinedField<NonNullable<T[P]>> }
export type DeepReadonly<T> = T extends Function
  ? T
  : T extends object
  ? { readonly [K in keyof T]: DeepReadonly<T[K]> }
  : T
export type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>

export const sleep = (time: number) => {
  return new Promise((resolve) => setTimeout(resolve, time))
}

export const OUT_OF_CANVAS_COORDINATES = {
  x: 100,
  y: 100,
  angle: 0
}

export const FORMATION_1 = {
  control: {
    teleportBall: {
      x: '0.000',
      y: '0.000',
      z: '0.021',
      vx: '0.000',
      vy: '0.000',
      vz: '0.000'
    },
    teleportRobot: [
      {
        id: {
          id: 0,
          team: 2
        },
        x: '-1.500',
        y: '1.120',
        orientation: '0.000'
      },
      {
        id: {
          id: 1,
          team: 2
        },
        x: '-1.500',
        y: '-0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 2,
          team: 2
        },
        x: '-1.500',
        y: '-1.120',
        orientation: '0.000'
      },
      {
        id: {
          id: 3,
          team: 2
        },
        x: '-0.550',
        y: '0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 4,
          team: 2
        },
        x: '-2.500',
        y: '-0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 5,
          team: 2
        },
        x: '-3.600',
        y: '-0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 6,
          team: 2
        },
        x: '-3.200',
        y: '0.750',
        orientation: '0.000'
      },
      {
        id: {
          id: 7,
          team: 2
        },
        x: '-3.200',
        y: '-0.750',
        orientation: '-0.001'
      },
      {
        id: {
          id: 8,
          team: 2
        },
        x: '-3.200',
        y: '1.500',
        orientation: '0.000'
      },
      {
        id: {
          id: 9,
          team: 2
        },
        x: '-3.198',
        y: '-1.500',
        orientation: '-0.012'
      },
      {
        id: {
          id: 10,
          team: 2
        },
        x: '-3.200',
        y: '2.250',
        orientation: '0.000'
      },
      {
        id: {
          id: 0,
          team: 1
        },
        x: '1.500',
        y: '1.120',
        orientation: '3.142'
      },
      {
        id: {
          id: 1,
          team: 1
        },
        x: '1.500',
        y: '0.000',
        orientation: '-3.142'
      },
      {
        id: {
          id: 2,
          team: 1
        },
        x: '1.500',
        y: '-1.120',
        orientation: '3.142'
      },
      {
        id: {
          id: 3,
          team: 1
        },
        x: '0.550',
        y: '-0.000',
        orientation: '-3.142'
      },
      {
        id: {
          id: 4,
          team: 1
        },
        x: '2.500',
        y: '0.000',
        orientation: '3.142'
      },
      {
        id: {
          id: 5,
          team: 1
        },
        x: '3.600',
        y: '0.000',
        orientation: '3.142'
      },
      {
        id: {
          id: 6,
          team: 1
        },
        x: '3.200',
        y: '0.750',
        orientation: '3.142'
      },
      {
        id: {
          id: 7,
          team: 1
        },
        x: '3.200',
        y: '-0.750',
        orientation: '3.142'
      },
      {
        id: {
          id: 8,
          team: 1
        },
        x: '3.200',
        y: '1.500',
        orientation: '3.142'
      },
      {
        id: {
          id: 9,
          team: 1
        },
        x: '3.200',
        y: '-1.500',
        orientation: '3.142'
      },
      {
        id: {
          id: 10,
          team: 1
        },
        x: '3.200',
        y: '2.250',
        orientation: '3.142'
      }
    ]
  }
}

export const FORMATION_2 = {
  control: {
    teleportBall: {
      x: '0.000',
      y: '0.000',
      z: '0.021',
      vx: '0.000',
      vy: '0.000',
      vz: '0.000'
    },
    teleportRobot: [
      {
        id: {
          id: 0,
          team: 2
        },
        x: '-4.200',
        y: '-0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 1,
          team: 2
        },
        x: '-3.400',
        y: '-0.200',
        orientation: '0.000'
      },
      {
        id: {
          id: 2,
          team: 2
        },
        x: '-3.400',
        y: '0.200',
        orientation: '0.000'
      },
      {
        id: {
          id: 3,
          team: 2
        },
        x: '-0.700',
        y: '-0.000',
        orientation: '0.000'
      },
      {
        id: {
          id: 4,
          team: 2
        },
        x: '-0.700',
        y: '2.250',
        orientation: '0.000'
      },
      {
        id: {
          id: 5,
          team: 2
        },
        x: '-0.700',
        y: '-2.250',
        orientation: '0.000'
      },
      {
        id: {
          id: 6,
          team: 2
        },
        x: '-2.000',
        y: '0.750',
        orientation: '0.000'
      },
      {
        id: {
          id: 7,
          team: 2
        },
        x: '-2.000',
        y: '-0.750',
        orientation: '0.000'
      },
      {
        id: {
          id: 8,
          team: 2
        },
        x: '-2.000',
        y: '1.500',
        orientation: '0.000'
      },
      {
        id: {
          id: 9,
          team: 2
        },
        x: '-2.000',
        y: '-1.500',
        orientation: '0.000'
      },
      {
        id: {
          id: 10,
          team: 2
        },
        x: '-2.000',
        y: '2.250',
        orientation: '0.000'
      },
      {
        id: {
          id: 0,
          team: 1
        },
        x: '4.200',
        y: '-0.000',
        orientation: '-3.141'
      },
      {
        id: {
          id: 1,
          team: 1
        },
        x: '3.400',
        y: '-0.200',
        orientation: '3.141'
      },
      {
        id: {
          id: 2,
          team: 1
        },
        x: '3.400',
        y: '0.200',
        orientation: '3.141'
      },
      {
        id: {
          id: 3,
          team: 1
        },
        x: '0.700',
        y: '0.000',
        orientation: '3.141'
      },
      {
        id: {
          id: 4,
          team: 1
        },
        x: '0.700',
        y: '2.250',
        orientation: '-3.141'
      },
      {
        id: {
          id: 5,
          team: 1
        },
        x: '0.700',
        y: '-2.250',
        orientation: '-3.141'
      },
      {
        id: {
          id: 6,
          team: 1
        },
        x: '2.000',
        y: '0.750',
        orientation: '-3.141'
      },
      {
        id: {
          id: 7,
          team: 1
        },
        x: '2.000',
        y: '-0.750',
        orientation: '-3.141'
      },
      {
        id: {
          id: 8,
          team: 1
        },
        x: '2.000',
        y: '1.500',
        orientation: '-3.141'
      },
      {
        id: {
          id: 9,
          team: 1
        },
        x: '2.000',
        y: '-1.500',
        orientation: '-3.141'
      },
      {
        id: {
          id: 10,
          team: 1
        },
        x: '2.000',
        y: '2.250',
        orientation: '-3.141'
      }
    ]
  }
}

export const robotNameMap = (team: 'BLACK' | 'PURPLE', id: number) => {
  if (team === 'PURPLE') {
    return {
      1: 'Wall-E',
      2: 'R2D2',
      3: 'Tron',
      4: 'Marvin',
      5: 'Jarvis',
      6: 'Baymax',
      7: 'Noo-Noo',
      8: 'T-800',
      9: 'K-9',
      10: 'Bender',
      11: 'Holt',
      12: 'Chappie',
      13: 'TARS',
      14: '',
      15: 'Herman'
    }[id]
  }

  return ''
}

export const formatFloat = (pos?: number | null): string => {
  if (pos === undefined || pos === null) {
    return '0.00'
  }

  const value = Math.abs(pos).toFixed(2)
  const sign = pos > -0.0001 ? '+' : '-'
  return `${sign}${value}`
}

export const sslRefCommandToString = (command: SSL_Referee.Command): string => {
  switch (command) {
    case SSL_Referee.Command.HALT:
      return 'Halt'
    case SSL_Referee.Command.STOP:
      return 'Stop'
    case SSL_Referee.Command.NORMAL_START:
      return 'Normal Start'
    case SSL_Referee.Command.FORCE_START:
      return 'Force Start'
    case SSL_Referee.Command.PREPARE_KICKOFF_YELLOW:
      return 'Prepare Kickoff Yellow'
    case SSL_Referee.Command.PREPARE_KICKOFF_BLUE:
      return 'Prepare Kickoff Blue'
    case SSL_Referee.Command.PREPARE_PENALTY_YELLOW:
      return 'Prepare Penalty Yellow'
    case SSL_Referee.Command.PREPARE_PENALTY_BLUE:
      return 'Prepare Penalty Blue'
    case SSL_Referee.Command.DIRECT_FREE_YELLOW:
      return 'Direct Free Yellow'
    case SSL_Referee.Command.DIRECT_FREE_BLUE:
      return 'Direct Free Blue'
    case SSL_Referee.Command.INDIRECT_FREE_YELLOW:
      return 'Indirect Free Yellow'
    case SSL_Referee.Command.INDIRECT_FREE_BLUE:
      return 'Indirect Free Blue'
    case SSL_Referee.Command.TIMEOUT_YELLOW:
      return 'Timeout Yellow'
    case SSL_Referee.Command.TIMEOUT_BLUE:
      return 'Timeout Blue'
    case SSL_Referee.Command.BALL_PLACEMENT_YELLOW:
      return 'Ball Placement Yellow'
    case SSL_Referee.Command.BALL_PLACEMENT_BLUE:
      return 'Ball Placement Blue'
    default:
      return 'Unknown'
  }
}

export const sslRefCommandToIconName = (command: SSL_Referee.Command): string => {
  switch (command) {
    case SSL_Referee.Command.HALT:
      return 'fa-stop'
    case SSL_Referee.Command.STOP:
      return 'fa-pause'
    case SSL_Referee.Command.NORMAL_START:
      return 'fa-play'
    case SSL_Referee.Command.FORCE_START:
      return 'fa-play-circle'
    case SSL_Referee.Command.PREPARE_KICKOFF_YELLOW:
      return 'fa-flag'
    case SSL_Referee.Command.PREPARE_KICKOFF_BLUE:
      return 'fa-flag'
    case SSL_Referee.Command.PREPARE_PENALTY_YELLOW:
      return 'fa-exclamation-triangle'
    case SSL_Referee.Command.PREPARE_PENALTY_BLUE:
      return 'fa-exclamation-triangle'
    case SSL_Referee.Command.DIRECT_FREE_YELLOW:
      return 'fa-arrow-right'
    case SSL_Referee.Command.DIRECT_FREE_BLUE:
      return 'fa-arrow-right'
    case SSL_Referee.Command.INDIRECT_FREE_YELLOW:
      return 'fa-arrow-up'
    case SSL_Referee.Command.INDIRECT_FREE_BLUE:
      return 'fa-arrow-up'
    case SSL_Referee.Command.TIMEOUT_YELLOW:
      return 'fa-clock'
    case SSL_Referee.Command.TIMEOUT_BLUE:
      return 'fa-clock'
    case SSL_Referee.Command.BALL_PLACEMENT_YELLOW:
      return 'fa-soccer-ball-o'
    case SSL_Referee.Command.BALL_PLACEMENT_BLUE:
      return 'fa-soccer-ball-o'
    default:
      return 'fa-question-circle'
  }
}

export const stageToString = (stage: Stage): string => {
  switch (stage) {
    case Stage.NORMAL_FIRST_HALF_PRE:
      return 'Normal First Half (Pre-Match)'
    case Stage.NORMAL_FIRST_HALF:
      return 'Normal First Half'
    case Stage.NORMAL_HALF_TIME:
      return 'Normal Half Time'
    case Stage.NORMAL_SECOND_HALF_PRE:
      return 'Normal Second Half (Pre-Match)'
    case Stage.NORMAL_SECOND_HALF:
      return 'Normal Second Half'
    case Stage.EXTRA_TIME_BREAK:
      return 'Extra Time Break'
    case Stage.EXTRA_FIRST_HALF_PRE:
      return 'Extra First Half (Pre-Match)'
    case Stage.EXTRA_FIRST_HALF:
      return 'Extra First Half'
    case Stage.EXTRA_HALF_TIME:
      return 'Extra Half Time'
    case Stage.EXTRA_SECOND_HALF_PRE:
      return 'Extra Second Half (Pre-Match)'
    case Stage.EXTRA_SECOND_HALF:
      return 'Extra Second Half'
    case Stage.PENALTY_SHOOTOUT_BREAK:
      return 'Penalty Shootout Break'
    case Stage.PENALTY_SHOOTOUT:
      return 'Penalty Shootout'
    case Stage.POST_GAME:
      return 'Post-Game'
    default:
      return 'Unknown'
  }
}
