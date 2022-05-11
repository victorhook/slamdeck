  -- Protocol --

PC is master, slamdeck (S) is slave.
A sample value is sent as 16-bit unsigned int.

## Read data from 1 sensor
- PC
| SLAMDECK_CMD_READ_SENSOR_X | sensor |
| 1 byte | 1 byte |

- Slamdeck
| data size | data |
| 1 byte    | 32-128 bytes |

## Read data from all sensors
- PC
| SLAMDECK_CMD_READ_SENSORS_ALL |
| 1 byte |

- Slamdeck
| data size | data |
| 2 byte    | 160-640 bytes |

## Set resolution
- PC
| SLAMDECK_CMD_SET_RESOLUTION | resolution |
| 1 byte | 1 byte |

- Slamdeck
| Request OK/Not ok |
| 1 byte |

## Set ranging frequency
- PC
| SLAMDECK_CMD_SET_RANGING_FREQUENCY | frequency |
| 1 byte | 1 byte |

- Slamdeck
| Request OK/Not ok |
| 1 byte |

## Set ranging mode
- PC
| SLAMDECK_CMD_SET_RANGING_MODE | mode |
| 1 byte | 1 byte |

- Slamdeck
| Request OK/Not ok |
| 1 byte |

- PC
| SLAMDECK_CMD_SET_INTEGRATION_TIME | integration time |
| 1 byte | 1 byte |

- Slamdeck
| Request OK/Not ok |
| 1 byte |