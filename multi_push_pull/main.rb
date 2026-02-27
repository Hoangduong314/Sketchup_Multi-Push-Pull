require 'sketchup.rb'

module YourName
  module MultiPushPull

    class MultiPushPullTool
      
      def activate
        @model = Sketchup.active_model
        @selection = @model.selection.grep(Sketchup::Face).to_a
        @needs_selection = @selection.empty? 
        @hover_face = nil 

        @state = 0 
        @start_pt = nil
        @distance = 0.0
        
        @mode = :follow 
        @keep_original = false 
        @is_isolated = false
        
        @mouse_x = 0
        @mouse_y = 0
        
        @ip = Sketchup::InputPoint.new
        @start_ip = Sketchup::InputPoint.new
        
        # Nếu đã chọn mặt từ trước (đang ở trong Group hoặc ngoài root)
        if !@needs_selection
          @selection_tr = @model.edit_transform # Lấy toạ độ ngữ cảnh hiện tại
          @ref_normal = @selection[0].normal
          calculate_joint_vectors()
        end

        UI.set_cursor(632) 
        update_status_text()
      end

      def update_status_text
        type_text = case @mode
                    when :follow then "Tiếp diễn (Follow)"
                    when :individual then "Riêng (Individual)"
                    when :joint then "Chung (Joint)"
                    end
        
        if @state == 0 && @needs_selection
          Sketchup.set_status_text("Click vào một mặt phẳng bất kỳ (kể cả trong Group) để bắt đầu", SB_VCB_LABEL)
        else
          keep_text = @keep_original ? "Bật" : "Tắt"
          Sketchup.set_status_text("Khoảng cách (Ctrl: Giữ mặt & cạnh gốc [#{keep_text}] | Tab: #{type_text})", SB_VCB_LABEL)
          Sketchup.set_status_text(@distance.to_l.to_s, SB_VCB_VALUE) if @state == 1
        end
      end

      def enableVCB?
        return true
      end

      def onCancel(reason, view)
        reset_tool(view)
      end

      def onKeyDown(key, repeat, flags, view)
        if key == COPY_MODIFIER_KEY
          @keep_original = !@keep_original
          update_status_text()
          view.invalidate
        elsif key == 9 
          if @mode == :follow
            @mode = :individual
          elsif @mode == :individual
            @mode = :joint
          else
            @mode = :follow
          end
          @keep_original = false 
          update_status_text()
          view.invalidate
        end
      end

      def edge_angle_less_than_180?(edge, face_s, face_u)
        return false unless face_s && face_u
        n_s = face_s.normal
        n_u = face_u.normal
        return false if n_s.dot(n_u) > 0.999
        edge_vec = edge.start.position.vector_to(edge.end.position)
        edge_vec.reverse! if edge.reversed_in?(face_s)
        inward_vec = n_s.cross(edge_vec)
        return n_u.dot(inward_vec) < -1e-5
      end

      def solve_3x3(m, c)
        det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1]) -
              m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
              m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0])
        return nil if det.abs < 1e-10
        inv_det = 1.0 / det
        inv = [
          [ (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * inv_det, (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * inv_det, (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv_det ],
          [ (m[1][2]*m[2][0] - m[1][0]*m[2][2]) * inv_det, (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv_det, (m[0][2]*m[1][0] - m[0][0]*m[1][2]) * inv_det ],
          [ (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * inv_det, (m[0][1]*m[2][0] - m[0][0]*m[2][1]) * inv_det, (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv_det ]
        ]
        vx = inv[0][0]*c[0] + inv[0][1]*c[1] + inv[0][2]*c[2]
        vy = inv[1][0]*c[0] + inv[1][1]*c[1] + inv[1][2]*c[2]
        vz = inv[2][0]*c[0] + inv[2][1]*c[1] + inv[2][2]*c[2]
        Geom::Vector3d.new(vx, vy, vz)
      end

      def solve_matrix(matrix_rows, b_values, sel_faces)
        ata = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        atb = [0.0, 0.0, 0.0]
        
        matrix_rows.each_with_index do |n, i|
          b = b_values[i]
          3.times do |r|
            atb[r] += n[r] * b
            3.times { |c| ata[r][c] += n[r] * n[c] }
          end
        end
        
        unique_eq_normals = []
        matrix_rows.each { |n| unique_eq_normals << n unless unique_eq_normals.any? { |un| n.parallel?(un) } }
        
        if unique_eq_normals.size == 2
          cross = unique_eq_normals[0].cross(unique_eq_normals[1])
          if cross.valid?
            cross.normalize!
            3.times do |r|
              atb[r] += cross[r] * 0.0
              3.times { |c| ata[r][c] += cross[r] * cross[c] }
            end
          end
        elsif unique_eq_normals.size == 1
          axes = [Geom::Vector3d.new(1,0,0), Geom::Vector3d.new(0,1,0), Geom::Vector3d.new(0,0,1)]
          axes.sort_by! { |ax| ax.dot(unique_eq_normals[0]).abs }
          t1 = unique_eq_normals[0].cross(axes[0]).normalize!
          t2 = unique_eq_normals[0].cross(t1).normalize!
          [t1, t2].each do |t|
            3.times do |r|
              atb[r] += t[r] * 0.0
              3.times { |c| ata[r][c] += t[r] * t[c] }
            end
          end
        end
        
        vec = solve_3x3(ata, atb)
        if vec.nil? || !vec.valid?
          sum_n = Geom::Vector3d.new(0,0,0)
          sel_faces.each { |f| sum_n += f.normal }
          sum_n.length > 0 ? sum_n.normalize! : sum_n = sel_faces.first.normal.clone
          dot_sum = 0.0
          sel_faces.each { |f| dot_sum += sum_n.dot(f.normal) }
          avg_dot = dot_sum / sel_faces.length
          mult = avg_dot.abs > 0.01 ? (1.0 / avg_dot) : 1.0
          vec = Geom::Vector3d.new(sum_n.x * mult, sum_n.y * mult, sum_n.z * mult)
        end
        vec
      end

      def calculate_joint_vectors
        @vertex_vectors = {}
        @follow_vectors = {} 
        @boundary_edges = []
        @internal_edges = []

        selected_edges = []
        @selection.each { |f| selected_edges.concat(f.edges) }
        selected_edges.uniq.each do |edge|
          faces_using_edge = edge.faces & @selection
          if faces_using_edge.length == 1
            @boundary_edges << edge 
          elsif faces_using_edge.length > 1
            @internal_edges << edge 
          end
        end

        vertices = @selection.map(&:vertices).flatten.uniq
        unselected_connected_faces = vertices.map(&:faces).flatten.uniq - @selection
        @is_isolated = unselected_connected_faces.empty?

        vertices.each do |v|
          sel_faces = v.faces & @selection
          unsel_faces = v.faces - @selection
          
          unique_sel_normals = []
          sel_faces.each { |f| unique_sel_normals << f.normal unless unique_sel_normals.any? { |n| n.parallel?(f.normal) } }

          unique_unsel_faces = []
          unsel_faces.each { |f| unique_unsel_faces << f unless unique_unsel_faces.any? { |uf| uf.normal.parallel?(f.normal) } }
          
          # Mode Joint Vectors
          matrix_rows_j = []
          b_values_j = []
          unique_sel_normals.each { |n| matrix_rows_j << n; b_values_j << 1.0 }
          
          v.edges.each do |e|
            f_s = (e.faces & @selection).first
            f_u = (e.faces - @selection).first
            if f_s && f_u && edge_angle_less_than_180?(e, f_s, f_u)
              matrix_rows_j << f_u.normal
              b_values_j << 0.0
            end
          end
          @vertex_vectors[v] = solve_matrix(matrix_rows_j, b_values_j, sel_faces)

          # Mode Follow Vectors
          matrix_rows_f = []
          b_values_f = []
          unique_sel_normals.each { |n| matrix_rows_f << n; b_values_f << 1.0 }
          
          unsel_count = unique_unsel_faces.size
          if unsel_count == 2
            unsel_f1 = unique_unsel_faces[0]
            unsel_f2 = unique_unsel_faces[1]
            
            common_edges = unsel_f1.edges & unsel_f2.edges
            unsel_edge = common_edges.find { |e| e.vertices.include?(v) }
            
            if unsel_edge
              phys_dir = v.position.vector_to(unsel_edge.other_vertex(v).position).normalize
              is_completely_in_negative = true
              unique_sel_normals.each do |sn|
                if phys_dir.dot(sn) > 1e-4 
                  is_completely_in_negative = false
                  break
                end
              end
              
              if is_completely_in_negative
                matrix_rows_f << unsel_f1.normal; b_values_f << 0.0
                matrix_rows_f << unsel_f2.normal; b_values_f << 0.0
              end
            end
          end
          @follow_vectors[v] = solve_matrix(matrix_rows_f, b_values_f, sel_faces)
        end
      end

      def create_side_faces(ents, pt1, pt2, pt3, pt4, ref_norm)
        faces = []
        begin
          face = ents.add_face(pt1, pt2, pt3, pt4)
          if face && face.valid?
            face.reverse! if face.normal.dot(ref_norm) < 0
            faces << face
            return faces
          end
        rescue => e
        end
        begin
          f1 = ents.add_face(pt1, pt2, pt3)
          if f1 && f1.valid?
            f1.reverse! if f1.normal.dot(ref_norm) < 0
            faces << f1
          end
          f2 = ents.add_face(pt1, pt3, pt4)
          if f2 && f2.valid?
            f2.reverse! if f2.normal.dot(ref_norm) < 0
            faces << f2
          end
          if f1 && f1.valid? && f2 && f2.valid?
            diag = (f1.edges & f2.edges).first
            if diag
              diag.soft = true
              diag.smooth = true
              diag.hidden = true
            end
          end
        rescue => e
        end
        faces
      end

      def create_cap_faces(ents, face, new_pts, target_norm)
        faces_created = []
        begin
          f = ents.add_face(new_pts)
          if f && f.valid?
            f.reverse! if f.normal.dot(target_norm) < 0
            faces_created << f
            return faces_created
          end
        rescue => e
        end
        begin
          mesh = face.mesh
          mesh.polygons.each do |poly|
            pts = poly.map do |idx|
              orig_pt = mesh.point_at(idx.abs)
              v_idx = face.vertices.index { |v| v.position.distance(orig_pt) < 1e-4 }
              v_idx ? new_pts[v_idx] : orig_pt
            end
            if pts.length >= 3
              f = ents.add_face(pts)
              if f && f.valid?
                f.reverse! if f.normal.dot(target_norm) < 0
                faces_created << f
              end
            end
          end
          faces_created.each do |f|
            f.edges.each do |e|
              if (e.faces & faces_created).length == 2
                e.soft = true; e.smooth = true; e.hidden = true
              end
            end
          end
        rescue => e
        end
        faces_created
      end

      def onLButtonDown(flags, x, y, view)
        if @state == 0
          @ip.pick(view, x, y)
          if @needs_selection
            @hover_face = @ip.face
            if @hover_face
              @selection = [@hover_face]
              @selection_tr = @ip.transformation # Lấy TRỰC TIẾP ma trận của Group/Component
              @ref_normal = @hover_face.normal
              calculate_joint_vectors()
              @start_pt = @ip.position # position này là toạ độ Global trên màn hình
              @state = 1
              update_status_text()
            end
          else
            @start_pt = @ip.position
            @state = 1
            update_status_text()
          end
        elsif @state == 1
          create_geometry()
          reset_tool(view)
        end
      end

      def onMouseMove(flags, x, y, view)
        @mouse_x = x
        @mouse_y = y

        if @state == 0
          @ip.pick(view, x, y)
          if @needs_selection
            @hover_face = @ip.face
            @selection_tr = @ip.transformation if @hover_face
          end
        else
          @ip.pick(view, x, y, @start_ip)
        end

        view.tooltip = @ip.tooltip if @ip.valid?

        if @state == 1 && @start_pt
          current_pt = @ip.position
          # Quy đổi điểm Global của chuột về Local để tính toán khoảng cách cực kỳ chính xác (tự động nhận Scale)
          local_start = @start_pt.transform(@selection_tr.inverse)
          local_curr = current_pt.transform(@selection_tr.inverse)
          move_vector = local_start.vector_to(local_curr)
          @distance = move_vector.valid? ? move_vector.dot(@ref_normal) : 0.0
          update_status_text()
        end

        view.invalidate 
      end

      def onUserText(text, view)
        return unless @state == 1
        begin
          input_distance = text.to_l
          
          # Quy đổi khoảng cách Global do người dùng nhập thành khoảng cách Local 
          # (Trường hợp Group bị Scale bóp méo, số nhập vào vẫn đúng kích thước thực)
          global_n = @ref_normal.transform(@selection_tr).normalize
          global_n.length = input_distance
          pt1 = Geom::Point3d.new(0, 0, 0)
          pt2 = pt1.offset(global_n)
          local_dist = pt1.transform(@selection_tr.inverse).distance(pt2.transform(@selection_tr.inverse))
          
          @distance = (@distance < 0 && input_distance > 0) ? -local_dist : local_dist
          create_geometry()
          reset_tool(view)
        rescue ArgumentError
          UI.beep
          puts "Lỗi: Giá trị nhập vào không phải là số hợp lệ!"
        end
      end

      def cleanup_coplanar_edges(ents, faces)
        valid_faces = faces.select(&:valid?)
        edges_to_check = valid_faces.flat_map(&:edges).uniq.select(&:valid?)
        edges_to_erase = []
        edges_to_check.each do |e|
          next unless e.valid? && e.faces.length == 2
          f1, f2 = e.faces
          edges_to_erase << e if f1.normal.samedirection?(f2.normal)
        end
        if @keep_original
          edges_to_erase.reject! { |e| @internal_edges.include?(e) || @boundary_edges.include?(e) }
        end
        ents.erase_entities(edges_to_erase) if edges_to_erase.any?
      end

      def create_geometry
        return if @distance.abs < 0.001 
        @model.start_operation('Multi Push Pull', true)
        
        # ĐẶC BIỆT: Ghi khối vào đúng Entities chứa mặt phẳng đó (Ví dụ: Chui tọt vào bên trong Group)
        ents = @selection.first.parent.entities
        
        begin
          user_keep_original = @keep_original
          @keep_original = true if @is_isolated

          affected_faces = case @mode
                           when :individual then create_geometry_individual(ents)
                           when :joint      then create_geometry_joint(ents)
                           when :follow     then create_geometry_follow(ents)
                           end

          if @is_isolated && @distance > 0
            @selection.each { |f| f.reverse! if f.valid? }
          end

          @keep_original = user_keep_original

          cleanup_coplanar_edges(ents, affected_faces) if affected_faces
          @model.commit_operation
        rescue Exception => e
          @model.abort_operation
          UI.messagebox("Đã xảy ra lỗi khi tạo khối: #{e.message}")
        end
      end

      def reset_tool(view)
        @state = 0
        @start_pt = nil
        @distance = 0.0
        @keep_original = false 
        @is_isolated = false
        @start_ip.clear 
        
        if @needs_selection
          @selection = []
          @hover_face = nil
          @vertex_vectors = {}
          @follow_vectors = {}
        end
        
        update_status_text()
        view.invalidate
      end

      def draw(view)
        @ip.draw(view) if @ip.valid?
        
        # Lấy Transformation (Cố định an toàn)
        tr = @selection_tr || Geom::Transformation.new

        if @state == 0 && @needs_selection && @hover_face && @hover_face.valid?
          pts = @hover_face.vertices.map { |v| v.position.transform(tr) }
          view.drawing_color = Sketchup::Color.new(0, 128, 255, 64) 
          view.draw(GL_POLYGON, pts)
          view.line_width = 2
          view.drawing_color = "blue"
          view.draw(GL_LINE_LOOP, pts)
        end

        if @keep_original
          cx, cy = @mouse_x + 16, @mouse_y + 16
          p1 = Geom::Point3d.new(cx - 5, cy, 0); p2 = Geom::Point3d.new(cx + 5, cy, 0)
          p3 = Geom::Point3d.new(cx, cy - 5, 0); p4 = Geom::Point3d.new(cx, cy + 5, 0)
          view.drawing_color = "black"
          view.line_width = 2
          view.draw2d(GL_LINES, [p1, p2, p3, p4])
        end

        return unless @state == 1 && @distance.abs > 0.001
        edge_color = @distance < 0 ? Sketchup::Color.new(255, 0, 0, 255) : Sketchup::Color.new(0, 255, 0, 255)     

        user_keep_original = @keep_original
        @keep_original = true if @is_isolated

        case @mode
        when :individual then draw_individual(view, edge_color, tr)
        when :joint      then draw_joint(view, edge_color, tr)
        when :follow     then draw_follow(view, edge_color, tr)
        end

        @keep_original = user_keep_original
      end
    end

    def self.start_tool
      Sketchup.active_model.select_tool(MultiPushPullTool.new)
    end

    unless file_loaded?(__FILE__)
      cmd = UI::Command.new("Joint Push Pull") { self.start_tool }
      cmd.menu_text = "Joint Push Pull"
      cmd.tooltip = "Joint Push Pull"
      cmd.status_bar_text = "Kéo/Đẩy nhiều mặt phẳng hoặc mặt cong cùng lúc."

      icon_dir = File.join(__dir__, 'icons')
      cmd.small_icon = File.join(icon_dir, "jpp_24.png")
      cmd.large_icon = File.join(icon_dir, "jpp_48.png")

      menu = UI.menu('Plugins')
      menu.add_item(cmd)

      toolbar = UI::Toolbar.new("My Custom Tools")
      toolbar.add_item(cmd)
      toolbar.show if toolbar.get_last_state == TB_NEVER_SHOWN

      file_loaded(__FILE__)
    end
  end
end

require_relative 'mode_individual'
require_relative 'mode_joint'
require_relative 'mode_follow'