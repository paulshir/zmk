## Personal ZMK Fork

Starts with this [base](https://github.com/paulshir/zmk/tree/split-serial-pr%2Bpio-led) as base. This includes
* [wired split over serial support PR](https://github.com/zmkfirmware/zmk/pull/2080)
* Zephyr branch with support for WS2812 on RP2040

Adds the following PRs:
* [Tri State](https://github.com/zmkfirmware/zmk/pull/1366)
* [Smart Word](https://github.com/zmkfirmware/zmk/pull/1451)

### Resetting this branch

```
git reset --hard paulshir/split-serial-pr+pio-led
git cherry-pick readme

git fetch upstream pull/1366/head:pr1366 --force
git fetch upstream pull/1451/head:pr1451 --force

git merge pr1366
git merge pr1451
```

### To update this file
```
git checkout readme
vim README.md
git add --update
git commit --fixup HEAD
git rebase -i --autosquash HEAD~2
```



