#!/bin/bash
# copy to src for using
cmd=$1
#if [ "$cmd" == "new_branch" ] || [ "$cmd" == "delete_branch" ] || [ "$cmd" == "push_branch" ]; then
branch_name=$2
#else
#  echo "Strings are not equal."
#fi

repositories=(
  "$(eval "pwd")""/collect_node/collect_node"
  "$(eval "pwd")""/collect_node/imu"
  "$(eval "pwd")""/collect_node/iot"
  "$(eval "pwd")""/collect_node/magnetometers"
  "$(eval "pwd")""/collect_node/mcu"
  "$(eval "pwd")""/collect_node/mcuota"
  "$(eval "pwd")""/collect_node/turbidity"
  "$(eval "pwd")""/collect_node/ulsound"
  "$(eval "pwd")""/collect_node/wf5803"
	"$(eval "pwd")""/collect_node/record_msg"
  "$(eval "pwd")""/collect_node/posttunnel"
  "$(eval "pwd")""/hj_interface"
  "$(eval "pwd")""/hj_manager"
  "$(eval "pwd")""/thirdparty"
  "$(eval "pwd")""/middleware_node/middleware_node"
  "$(eval "pwd")""/planning_node/planning_node"
  "$(eval "pwd")""/slam_node/slam_node"
  "$(eval "pwd")""/utils_node/utils_node"
)
new_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git checkout -b "$branch_name"
}

delete_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git branch -d "$branch_name"
}

push_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git push origin "$branch_name"
}
delete_remote_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git push origin --delete "$branch_name"
}
switch_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git checkout "$branch_name"
}

pull_branch() {
  local repo_path=$1
  local branch_name=$2
  cd "$repo_path"
  git pull origin "$branch_name"
}

create_tag() {
  local repo_path=$1
  local tag_name=$2
  cd "$repo_path"
  git tag "$tag_name"
}

push_tag() {
  local repo_path=$1
  local tag_name=$2
  cd "$repo_path"
  git push origin "$tag_name"
}
if [ "$cmd" == "new_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		new_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "delete_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		delete_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "push_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		push_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "delete_remote_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		delete_remote_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "switch_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		switch_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "pull_branch" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		pull_branch "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "create_tag" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		create_tag "$repo" "$branch_name"
	done
fi

if [ "$cmd" == "push_tag" ]; then
	for repo in "${repositories[@]}"; do
		echo "$repo"
		push_tag "$repo" "$branch_name"
	done
fi






