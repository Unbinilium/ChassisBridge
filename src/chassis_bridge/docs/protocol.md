# TCP Protocol

The default port is set to 60000.

## Receive Protocol

### Frame

```
Little Endian -> | 1 byte header | 2 bytes action_id | 4 bytes timestamp | 12 bytes volocity | 12 bytes acceleration | 12 bytes diffusion |
```

Binary format, totally 43 bytes.

### Types

- header `char`
- action_id `uint16_t`
- timestamp `uint32_t`
- volocity
    - x `float32`
    - y `float32`
    - z `float32`
- acceleration
    - x `float32`
    - y `float32`
    - z `float32`
- diffusion
    - x `float32`
    - y `float32`
    - z `float32`

### Detailes

- header - single char `S`
- action_id - current action id
- timestamp - chassis's unix time, precision microseconds (timestamp = int64_t(unix_time) % uint32_t)
- volocity
    - x - move volocity, unit m/s
    - y - move volocity, unit m/s
    - z - rotate volocity, unit rad/s
- acceleration
    - x - move acceleration, unit m/s^2
    - y - move acceleration, unit m/s^2
    - z - rotate acceleration, unit rad/s^2
- diffusion
    - x - moved distance on x axis since previous action, unit m
    - y - moved distance on y axis since previous action, unit m
    - z - rotated angle on z axis since previous action, unit rad


## Transmit Protocol

### Frame

```
Little Endian -> | 1 byte header | 2 bytes action_id | 4 bytes timestamp | 1 bytes action_type | 12 bytes tuple |
```

Binary format, totally 20 bytes.

### Types

- header `char`
- action_id `uint16_t`
- timestamp `uint32_t`
- action_type `char`
- tuple
    - x `float32`
    - y `float32`
    - z `float32`

### Detailes

- header - single char `T`
- action_id - current action id
- timestamp - computer's unix time, precision microseconds (timestamp = int64_t(unix_time) % int32_t)
- action_type - single char which specifies the types of the tuple data
    - `V`
        - x - target move volocity, unit m/s
        - y - target move volocity, unit m/s
        - z - target rotate volocity, unit rad/s
    - `A`
        - x - target move acceleration, unit m/s^2
        - y - target move acceleration, unit m/s^2
        - z - target rotate acceleration, unit rad/s^2
    - `D`
        - x - move distance on x axis, unit m
        - y - move distance on y axis, unit m
        - z - rotate angle on z axis, unit rad
