from dataclasses import dataclass, field

@dataclass
class IMUData:
    time: list = field(default_factory=list)

    ax: list = field(default_factory=list)
    ay: list = field(default_factory=list)
    az: list = field(default_factory=list)

    gx: list = field(default_factory=list)
    gy: list = field(default_factory=list)
    gz: list = field(default_factory=list)
    
    freq: float = 0.0