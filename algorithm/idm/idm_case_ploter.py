import bokeh
from bokeh.plotting import figure
from bokeh.layouts import column, row


class IDMCasePloter():
    def plot_idm_case(self, plot_data) -> bokeh.layouts.column:
        t_triger = plot_data["t_triger"]
        s_leader = plot_data["s_leader"]
        v_leader = plot_data["v_leader"]
        a_leader = plot_data["a_leader"]

        t_ref_out = plot_data["t_ref_out"]
        s_ref_out = plot_data["s_ref_out"]
        v_ref_out = plot_data["v_ref_out"]
        a_ref_out = plot_data["a_ref_out"]

        # input data
        fig_leader_v_s = figure(width=400, height=200, y_range=[0, 100])
        fig_leader_v_s.scatter(
            s_leader, v_leader, color='blue', line_alpha=0.4, legend_label="leader_v_s")
        fig_leader_v_s.scatter(
            s_ref_out, v_ref_out, color='black', line_alpha=0.4, legend_label="ego_idm_v_s")
        fig_leader_v_s.xaxis.axis_label = "s/m"
        fig_leader_v_s.yaxis.axis_label = "v/m/s"

        fig_leader_s_t = figure(width=400, height=200)
        fig_leader_s_t.scatter(
            t_triger, s_leader, color='red', line_alpha=0.4, legend_label="leader_s_t")
        fig_leader_s_t.xaxis.axis_label = "t/s"
        fig_leader_s_t.yaxis.axis_label = "s/m"

        fig_leader_a_t = figure(width=400, height=200, y_range=[-5, 3.5])
        fig_leader_a_t.scatter(
            t_triger, a_leader, color='red', line_alpha=0.4, legend_label="leader_a_t")
        fig_leader_a_t.xaxis.axis_label = "t/s"
        fig_leader_a_t.yaxis.axis_label = "a/mps2"

        fig_leader_v_t = figure(width=400, height=200, y_range=[-20, 100])
        fig_leader_v_t.scatter(
            t_triger, v_leader, color='red', line_alpha=0.4, legend_label="leader_v_t")
        fig_leader_v_t.xaxis.axis_label = "t/s"
        fig_leader_v_t.yaxis.axis_label = "v/kph"

        # output data
        fig_idm_s_t = figure(width=400, height=200)
        fig_idm_s_t.scatter(t_ref_out, s_ref_out, color='red',
                            line_alpha=0.4, legend_label="idm_s_t")
        fig_idm_s_t.xaxis.axis_label = "t/s"
        fig_idm_s_t.yaxis.axis_label = "s/m"

        fig_idm_a_t = figure(width=400, height=200, y_range=[-5, 3.5])
        fig_idm_a_t.scatter(t_ref_out, a_ref_out, color='red',
                            line_alpha=0.4, legend_label="idm_a_t")
        fig_idm_a_t.xaxis.axis_label = "t/s"
        fig_idm_a_t.yaxis.axis_label = "a/mps2"

        fig_idm_v_t = figure(width=400, height=200, y_range=[0, 100])
        fig_idm_v_t.scatter(t_ref_out, v_ref_out, color='red',
                            line_alpha=0.4, legend_label="idm_v_t")
        fig_idm_v_t.xaxis.axis_label = "t/s"
        fig_idm_v_t.yaxis.axis_label = "v/kph"

        relative_s_t = [x - y for x, y in zip(s_leader, s_ref_out)]
        fig_relative_s_t = figure(width=400, height=200)
        fig_relative_s_t.scatter(
            t_ref_out, relative_s_t, color='red', line_alpha=0.4, legend_label="leader-idm_s_t")
        fig_relative_s_t.xaxis.axis_label = "t/s"
        fig_relative_s_t.yaxis.axis_label = "relative_s/m"

        row1 = row(fig_leader_v_s, fig_leader_s_t,
                   fig_leader_a_t, fig_leader_v_t)
        row2 = row(fig_idm_s_t, fig_idm_a_t, fig_idm_v_t, fig_relative_s_t)
        layout_res = column(row1, row2)
        return layout_res