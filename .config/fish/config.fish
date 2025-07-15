function fish_prompt
    set -l last_status $status
    set -l normal (set_color normal)
    set -l user_color (set_color $fish_color_user)
    set -l host_color (set_color $fish_color_host)
    set -l cwd_color (set_color $fish_color_cwd)
    set -l git_color (set_color $fish_color_git)
    set -l venv ""
    if set -q VIRTUAL_ENV_PROMPT
        set venv "$VIRTUAL_ENV_PROMPT"
    end

    echo -n -s $venv $user_color (whoami) $host_color @ (prompt_hostname) ' ' $cwd_color (prompt_pwd) $normal (fish_git_prompt)
    if test $last_status -ne 0
        set_color red
        echo -n " [$last_status]"
    end
    set_color normal
    echo -n " > "
end
