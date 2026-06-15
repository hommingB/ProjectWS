# Robot State Machine (Mermaid.js)

Below is the Mermaid.js state diagram syntax representing how the `mode_manager` handles the robot's states. You can preview this directly in Markdown editors (like Obsidian, VS Code, or Notion) or copy the code block and paste it into [mermaid.live](https://mermaid.live/) to export it as an image.

```mermaid
stateDiagram-v2
    classDef stateStyle fill:#f8fafc,stroke:#64748b,stroke-width:1px,color:#0f172a,font-weight:bold;
    classDef pausedStyle fill:#fee2e2,stroke:#ef4444,stroke-width:1.5px,color:#991b1b,font-weight:bold;
    classDef activeStyle fill:#ecfdf5,stroke:#10b981,stroke-width:1.5px,color:#065f46,font-weight:bold;

    [*] --> PAUSED
    
    PAUSED:::pausedStyle --> IDLE : /resume
    IDLE:::stateStyle --> NAV_BUSY : Dispatch Nav Task
    
    NAV_BUSY:::activeStyle --> IDLE : Goal Reached (No Dwell)
    NAV_BUSY --> WAITING : Goal Reached (With Dwell)
    WAITING:::stateStyle --> IDLE : Dwell Timeout Expired
    
    IDLE --> DOCKING : Battery Low / /gotodock
    DOCKING:::stateStyle --> RESTING : Dock Reached
    RESTING:::stateStyle --> CHARGING : Charger Connected
    CHARGING:::activeStyle --> IDLE : Fully Charged / Wake
    
    %% Interruption / Failure paths
    NAV_BUSY --> PAUSED : /pause or Nav2 Fail
    DOCKING --> PAUSED : /pause or Dock Fail (Battery Low)
    DOCKING --> IDLE : Dock Fail (Battery OK)
    IDLE --> PAUSED : /pause
    WAITING --> PAUSED : /pause
```
