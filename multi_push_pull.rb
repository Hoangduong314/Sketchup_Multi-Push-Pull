require 'sketchup.rb'
require 'extensions.rb'

module YourName
  module MultiPushPull
    unless file_loaded?(__FILE__)
      ex = SketchupExtension.new('Multi Push Pull', 'multi_push_pull/main.rb')
      ex.description = 'Công cụ đẩy nhiều mặt phẳng nâng cao.'
      ex.version     = '1.0.0'
      ex.copyright   = 'Your Name © 2024'
      ex.creator     = 'Your Name'
      Sketchup.register_extension(ex, true)
      file_loaded(__FILE__)
    end
  end
end