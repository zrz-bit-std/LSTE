# SA-PPO policy

Place the trained SA-PPO checkpoint here before starting the controller:

```text
rl_navigation/policy/sa_peppo_1650.pth
```

Policy checkpoints are runtime artifacts and are intentionally excluded from
Git. The startup health check reports a clear error when the file is missing.
