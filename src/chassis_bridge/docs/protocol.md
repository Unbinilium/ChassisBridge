## Receive Protocol

### Frame

```
Little Endian -> | 1 byte header | 2 byte action_id | 4 byte timestamp | 12 byte volocity | 12 byte acceleration |
```

Binary format, totally 31 bytes.

### Types

- header `char`
- action_id `uint16_t`
- timestamp `float32`
- volocity
    - x `float32`
    - y `float32`
    - z `float32`
- acceleration
    - x `float32`
    - y `float32`
    - z `float32`

### Detailes

- header - single char `S`
- action_id - current action id
- timestamp - chassis's unix time, precision ms
- volocity
    - x - move volocity, unit m/s
    - y - move volocity, unit m/s
    - z - rotate volocity, unit rad/s
- acceleration
    - x - move acceleration, unit m/s^2
    - y - move acceleration, unit m/s^2
    - z - rotate acceleration, unit rad/s^2


## Transmit Protocol

### Frame

```
Little Endian -> | 1 byte header | 2 byte action_id | 4 byte timestamp | 1 byte action_type | 12 byte tuple |
```

Binary format, totally 20 bytes.

### Types

- header `char`
- action_id `uint16_t`
- timestamp `float32`
- action_type `char`
- tuple
    - x `float32`
    - y `float32`
    - z `float32`

### Detailes

- header - single char `T`
- action_id - current action id
- timestamp - computer's unix time, precision ms
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
